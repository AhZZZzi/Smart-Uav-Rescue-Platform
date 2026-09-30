"""
UAV Cloud Bridge Node - ENU Yaw Fix & Relative Altitude
Integrated with Precision Landing handoff.
"""

import asyncio
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from mavros_msgs.msg import State, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Header, Bool, String

import websockets

RELAY_URL = "wss://uav-rescue-relay.onrender.com/ws/drone"

# Safety thresholds for pre-flight check
MIN_BATTERY_PCT = 30.0
MIN_GPS_FIX_STATUS = 0
SERVICE_WAIT_TIMEOUT_SEC = 3.0
OFFBOARD_CONFIRM_TIMEOUT_SEC = 5.0
ARM_CONFIRM_TIMEOUT_SEC = 5.0
DEFAULT_ALTITUDE_M = 20.0
MIN_ALTITUDE_M = 5.0
MAX_ALTITUDE_M = 60.0
ARRIVAL_RADIUS_M = 3.0
ARRIVAL_CHECK_INTERVAL_SEC = 1.0
MISSION_TIMEOUT_SEC = 120.0
LAND_CONFIRM_TIMEOUT_SEC = 5.0
DISARM_CONFIRM_TIMEOUT_SEC = 30.0
PRECISION_LANDING_TIMEOUT_SEC = 30.0  # max time to wait for precision landing to finish


class CloudBridgeNode(Node):
    def __init__(self):
        super().__init__("uav_cloud_bridge")
        self.get_logger().info("UAV Cloud Bridge starting...")

        self.current_state = None
        self.current_battery = None
        self.current_gps = None
        self.current_landing_status = None

        self.ws = None
        self.ws_loop = None
        self.keep_streaming = False

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscriptions
        self.create_subscription(State, "/mavros/state", self.state_cb, 10)
        self.create_subscription(BatteryState, "/mavros/battery", self.battery_cb, sensor_qos)
        self.create_subscription(NavSatFix, "/mavros/global_position/global", self.gps_cb, sensor_qos)
        self.create_subscription(String, "/precision_landing/status", self.landing_status_cb, 10)

        # Clients
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # Publishers
        self.setpoint_pub = self.create_publisher(GlobalPositionTarget, "/mavros/setpoint_raw/global", 10)
        self.landing_activate_pub = self.create_publisher(Bool, "/precision_landing/activate", 10)

        self.create_timer(2.0, self.send_telemetry)

        # Start WebSocket
        self.ws_thread = threading.Thread(target=self._start_ws_loop, daemon=True)
        self.ws_thread.start()

        self.get_logger().info("Bridge node ready.")

    def state_cb(self, msg):
        self.current_state = msg

    def battery_cb(self, msg):
        self.current_battery = round(msg.percentage * 100, 1)

    def gps_cb(self, msg):
        self.current_gps = msg

    def landing_status_cb(self, msg):
        self.current_landing_status = msg.data
        self.get_logger().info(f"Precision landing status: {msg.data}")

    def send_telemetry(self):
        if not self.ws or not self.ws_loop:
            return

        telemetry = {
            "type": "telemetry",
            "armed": self.current_state.armed if self.current_state else False,
            "mode": self.current_state.mode if self.current_state else "UNKNOWN",
            "battery_pct": self.current_battery,
            "lat": self.current_gps.latitude if self.current_gps else None,
            "lon": self.current_gps.longitude if self.current_gps else None,
        }

        asyncio.run_coroutine_threadsafe(
            self._ws_send(json.dumps(telemetry)), self.ws_loop
        )

    def handle_dispatch(self, payload):
        lat = payload.get("lat")
        lon = payload.get("lon")

        alt = payload.get("altitude", DEFAULT_ALTITUDE_M)
        try:
            alt = float(alt)
        except (TypeError, ValueError):
            self.get_logger().warn(f"Invalid altitude value ({alt}), using default {DEFAULT_ALTITUDE_M}m")
            alt = DEFAULT_ALTITUDE_M

        clamped_alt = max(MIN_ALTITUDE_M, min(MAX_ALTITUDE_M, alt))
        if clamped_alt != alt:
            self.get_logger().warn(
                f"Altitude {alt}m outside safe range [{MIN_ALTITUDE_M}, {MAX_ALTITUDE_M}], "
                f"clamped to {clamped_alt}m"
            )

        self.get_logger().info(f"Dispatch received → lat={lat}, lon={lon}, altitude={clamped_alt}m")
        threading.Thread(target=self._arm_and_takeoff, args=(lat, lon, clamped_alt), daemon=True).start()

    def _preflight_check(self):
        if not self.current_state or not self.current_state.connected:
            return False, "FCU not connected (mavros/state.connected = False)"

        if self.current_state.armed:
            return False, "Drone is already ARMED, aborting dispatch to avoid conflict"

        if self.current_battery is None:
            return False, "No battery data received from FCU yet"
        if self.current_battery < MIN_BATTERY_PCT:
            return False, f"Battery too low ({self.current_battery}% < {MIN_BATTERY_PCT}%)"

        if self.current_gps is None:
            return False, "No GPS data received from FCU yet"
        if self.current_gps.status.status < MIN_GPS_FIX_STATUS:
            return False, f"No valid GPS fix (status={self.current_gps.status.status})"

        return True, "OK"

    def _arm_and_takeoff(self, lat, lon, altitude=DEFAULT_ALTITUDE_M):
        import time as _t

        ok, reason = self._preflight_check()
        if not ok:
            self.get_logger().error(f"Pre-flight check FAILED, aborting dispatch: {reason}")
            self._notify_status("preflight_failed", lat=lat, lon=lon, altitude=altitude)
            return
        self.get_logger().info("Pre-flight check passed.")

        self.get_logger().info(f"Initiating autonomous dispatch lifecycle for lat={lat}, lon={lon}, altitude={altitude}m")

        msg = GlobalPositionTarget()
        msg.header = Header()
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT

        msg.type_mask = (
            GlobalPositionTarget.IGNORE_VX |
            GlobalPositionTarget.IGNORE_VY |
            GlobalPositionTarget.IGNORE_VZ |
            GlobalPositionTarget.IGNORE_AFX |
            GlobalPositionTarget.IGNORE_AFY |
            GlobalPositionTarget.IGNORE_AFZ |
            GlobalPositionTarget.IGNORE_YAW_RATE
        )
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(altitude)

        self.keep_streaming = True

        def _proof_of_life_stream():
            self.get_logger().info("Proof of Life stream RUNNING at 10Hz...")
            while self.keep_streaming:
                msg.header.stamp = self.get_clock().now().to_msg()

                if self.current_gps is not None:
                    msg.yaw = self._bearing_rad(
                        self.current_gps.latitude, self.current_gps.longitude, float(lat), float(lon)
                    )
                else:
                    msg.yaw = 0.0

                self.setpoint_pub.publish(msg)
                _t.sleep(0.1)

        threading.Thread(target=_proof_of_life_stream, daemon=True).start()
        _t.sleep(2.0)

        if not self.mode_client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT_SEC):
            self.get_logger().error("Set_mode service not available, aborting dispatch.")
            self.keep_streaming = False
            return

        if not self.arming_client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT_SEC):
            self.get_logger().error("Arming service not available, aborting dispatch.")
            self.keep_streaming = False
            return

        self.get_logger().info("Switching to OFFBOARD mode...")
        offboard_req = SetMode.Request()
        offboard_req.custom_mode = "OFFBOARD"
        self.mode_client.call_async(offboard_req)

        if not self._wait_for_condition(
            lambda: self.current_state is not None and self.current_state.mode == "OFFBOARD",
            timeout_sec=OFFBOARD_CONFIRM_TIMEOUT_SEC,
        ):
            self.get_logger().error("Could not confirm OFFBOARD mode after timeout, aborting dispatch.")
            self.keep_streaming = False
            self._notify_status("offboard_failed", lat=lat, lon=lon, altitude=altitude)
            return
        self.get_logger().info("OFFBOARD mode confirmed.")

        self.get_logger().info("Sending Standard Arm command under OFFBOARD mode...")
        arm_req = CommandBool.Request()
        arm_req.value = True
        self.arming_client.call_async(arm_req)

        if not self._wait_for_condition(
            lambda: self.current_state is not None and self.current_state.armed,
            timeout_sec=ARM_CONFIRM_TIMEOUT_SEC,
        ):
            self.get_logger().error("Arming denied or not confirmed after timeout.")
            self.keep_streaming = False
            self._notify_status("arm_failed", lat=lat, lon=lon, altitude=altitude)
            return

        self.get_logger().info("Drone ARMED and flying autonomously via OFFBOARD!")
        self._notify_status("armed", lat=lat, lon=lon, altitude=altitude)

        start_time = _t.time()
        arrived = False
        while _t.time() - start_time < MISSION_TIMEOUT_SEC:
            if self.current_gps is not None:
                dist = self._haversine_m(
                    self.current_gps.latitude, self.current_gps.longitude, lat, lon
                )
                if dist <= ARRIVAL_RADIUS_M:
                    arrived = True
                    break
            _t.sleep(ARRIVAL_CHECK_INTERVAL_SEC)

        self.keep_streaming = False  # stop proof-of-life stream before handing off

        if not arrived:
            self.get_logger().warn("Arrival not confirmed (timeout), stopping tracking.")
            self._notify_status("timeout", lat=lat, lon=lon, altitude=altitude)
            self.get_logger().info("Mission timeline finished.")
            return

        self.get_logger().info(f"Arrived at target (within {ARRIVAL_RADIUS_M}m radius).")
        self._notify_status("arrived", lat=lat, lon=lon, altitude=altitude)

        # --- Hand off to Precision Landing ---
        self.get_logger().info("Activating precision landing...")
        self.current_landing_status = None
        self.landing_activate_pub.publish(Bool(data=True))
        self._notify_status("precision_landing_active", lat=lat, lon=lon, altitude=altitude)

        landing_start = _t.time()
        landing_succeeded = False

        while _t.time() - landing_start < PRECISION_LANDING_TIMEOUT_SEC:
            if self.current_landing_status == "LANDED":
                landing_succeeded = True
                break
            if self.current_landing_status == "FAILED":
                break
            _t.sleep(0.5)

        if landing_succeeded:
            self.get_logger().info("Precision landing succeeded.")
            self._notify_status("landed_precision", lat=lat, lon=lon, altitude=altitude)
        else:
            self.get_logger().warn(
                f"Precision landing did not complete (status={self.current_landing_status}). "
                "Falling back to AUTO.LAND (GPS blind landing)."
            )
            self.landing_activate_pub.publish(Bool(data=False))

            land_req = SetMode.Request()
            land_req.custom_mode = "AUTO.LAND"
            self.mode_client.call_async(land_req)

            if self._wait_for_condition(
                lambda: self.current_state is not None and self.current_state.mode == "AUTO.LAND",
                timeout_sec=LAND_CONFIRM_TIMEOUT_SEC,
            ):
                self._notify_status("landing_fallback", lat=lat, lon=lon, altitude=altitude)
            else:
                self.get_logger().error("Could not confirm AUTO.LAND fallback mode.")
                self._notify_status("land_failed", lat=lat, lon=lon, altitude=altitude)

            if self._wait_for_condition(
                lambda: self.current_state is not None and not self.current_state.armed,
                timeout_sec=DISARM_CONFIRM_TIMEOUT_SEC,
            ):
                self.get_logger().info("Touchdown confirmed — drone has disarmed.")
                self._notify_status("landed_fallback", lat=lat, lon=lon, altitude=altitude)
            else:
                self.get_logger().warn("Drone still armed after timeout in AUTO.LAND fallback.")
                self._notify_status("land_timeout", lat=lat, lon=lon, altitude=altitude)

        self.get_logger().info("Mission timeline finished.")

    @staticmethod
    def _bearing_rad(lat1, lon1, lat2, lon2):
        dLon = math.radians(lon2 - lon1)
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)

        y = math.sin(dLon) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dLon)
        compass_bearing = math.atan2(y, x)

        enu_yaw = (math.pi / 2.0) - compass_bearing

        if enu_yaw > math.pi:
            enu_yaw -= 2.0 * math.pi
        elif enu_yaw < -math.pi:
            enu_yaw += 2.0 * math.pi

        return enu_yaw

    @staticmethod
    def _haversine_m(lat1, lon1, lat2, lon2):
        R = 6371000.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        return 2 * R * math.asin(math.sqrt(a))

    def _notify_status(self, status, lat=None, lon=None, altitude=None):
        if not self.ws or not self.ws_loop:
            return
        message = {
            "type": "mission_status",
            "status": status,
            "target_lat": lat,
            "target_lon": lon,
            "target_altitude": altitude,
            "current_lat": self.current_gps.latitude if self.current_gps else None,
            "current_lon": self.current_gps.longitude if self.current_gps else None,
        }
        asyncio.run_coroutine_threadsafe(
            self._ws_send(json.dumps(message)), self.ws_loop
        )

    def _wait_for_condition(self, predicate, timeout_sec, poll_interval_sec=0.2):
        import time as _t
        deadline = _t.time() + timeout_sec
        while _t.time() < deadline:
            if predicate():
                return True
            _t.sleep(poll_interval_sec)
        return predicate()

    def _start_ws_loop(self):
        self.ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.ws_loop)
        self.ws_loop.run_until_complete(self._ws_connect_loop())

    async def _ws_connect_loop(self):
        while True:
            try:
                async with websockets.connect(RELAY_URL) as ws:
                    self.ws = ws
                    self.get_logger().info("Connected to relay server.")
                    async with ws:
                        async for message in ws:
                            payload = json.loads(message)
                            if payload.get("type") == "dispatch":
                                self.handle_dispatch(payload)
            except Exception as e:
                self.ws = None
                self.get_logger().warn(f"Relay disconnected: {e}. Reconnecting...")
                await asyncio.sleep(5)

    async def _ws_send(self, message: str):
        if self.ws:
            try:
                await self.ws.send(message)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = CloudBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()