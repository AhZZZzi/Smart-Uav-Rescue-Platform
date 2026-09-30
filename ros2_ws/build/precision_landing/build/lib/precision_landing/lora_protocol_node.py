"""
LoRa Protocol Node (companion computer side, drone role).
Sends compact telemetry over the (mock) LoRa transport when the primary
communication channel (WebSocket to relay server) is degraded or lost.
Uses lora_transport_mock.MockLoraTransport; swap for real hardware driver later.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import String, Bool
import time

from precision_landing.lora_transport_mock import MockLoraTransport

MSG_TYPE_TELEMETRY = 0
MSG_TYPE_ALERT = 1

TELEMETRY_SEND_INTERVAL_SEC = 2.0  # low rate, appropriate for LoRa bandwidth


class LoraProtocolNode(Node):
    def __init__(self):
        super().__init__('lora_protocol_node')

        self.transport = MockLoraTransport(node_role="drone")

        self.current_gps = None
        self.current_battery = None
        self.current_state = None
        self.primary_link_ok = True  # updated externally (e.g. from bridge_node's WS status)

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_cb, sensor_qos)
        self.create_subscription(BatteryState, '/mavros/battery', self.battery_cb, sensor_qos)
        self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.create_subscription(Bool, '/comms/primary_link_status', self.link_status_cb, 10)

        self.status_pub = self.create_publisher(String, '/lora/status', 10)

        self.create_timer(TELEMETRY_SEND_INTERVAL_SEC, self.send_telemetry_if_needed)

        self.get_logger().info("LoRa Protocol Node started (mock transport, drone role)")

    def gps_cb(self, msg):
        self.current_gps = msg

    def battery_cb(self, msg):
        self.current_battery = msg

    def state_cb(self, msg):
        self.current_state = msg

    def link_status_cb(self, msg: Bool):
        """External signal: True = primary WebSocket link is healthy, False = degraded/lost."""
        was_ok = self.primary_link_ok
        self.primary_link_ok = msg.data
        if was_ok and not self.primary_link_ok:
            self.get_logger().warn("Primary link degraded. Falling back to LoRa telemetry.")
            self.status_pub.publish(String(data="LORA_ACTIVE"))
        elif not was_ok and self.primary_link_ok:
            self.get_logger().info("Primary link restored. LoRa telemetry paused.")
            self.status_pub.publish(String(data="LORA_STANDBY"))

    def send_telemetry_if_needed(self):
        if self.primary_link_ok:
            return

        if self.current_gps is None:
            self.get_logger().warn("No GPS data yet, skipping LoRa telemetry send.")
            return

        battery_pct = int(self.current_battery.percentage * 100) if self.current_battery else 0
        armed = self.current_state.armed if self.current_state else False

        status_flags = 0
        if armed:
            status_flags |= 0b0001
        if self.current_gps.status.status >= 0:
            status_flags |= 0b0010

        self.transport.send_packet(
            msg_type=MSG_TYPE_TELEMETRY,
            timestamp=int(time.time()),
            lat=self.current_gps.latitude,
            lon=self.current_gps.longitude,
            alt=self.current_gps.altitude,
            battery_pct=battery_pct,
            status_flags=status_flags,
        )
        self.get_logger().info(
            f"LoRa telemetry sent: lat={self.current_gps.latitude:.6f}, "
            f"lon={self.current_gps.longitude:.6f}, battery={battery_pct}%"
        )

    def destroy_node(self):
        self.transport.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoraProtocolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()