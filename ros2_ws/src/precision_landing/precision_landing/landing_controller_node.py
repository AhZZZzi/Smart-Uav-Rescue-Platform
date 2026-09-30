"""
Precision Landing Controller Node
Activated by bridge node via /precision_landing/activate.
Subscribes to ArUco marker pose, runs a state machine,
publishes velocity setpoints to MAVROS2, then hands off
final touchdown to PX4's AUTO.LAND mode.
Reports progress back via /precision_landing/status.

Key behaviors:
- Phase 1 (Tracking): OFFBOARD velocity control guided by ArUco marker pose.
- Grace period: if marker is briefly lost, hold the LAST known setpoint
  (instead of freezing to zero or free-drifting) to avoid jerky motion.
- Phase 2 (Final Landing): when Z < threshold, hand off to FCU AUTO.LAND
  instead of trying to push OFFBOARD control down to the ground, which
  is prone to ground-effect turbulence.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from std_msgs.msg import Bool, String
import time

# --- State machine states ---
STATE_IDLE = "IDLE"
STATE_SEARCHING = "SEARCHING"
STATE_ALIGNING = "ALIGNING"
STATE_DESCENDING = "DESCENDING"
STATE_AUTO_LANDING = "AUTO_LANDING"
STATE_LANDED = "LANDED"
STATE_FAILED = "FAILED"

# --- Tunable parameters ---
XY_ALIGN_TOLERANCE = 0.15
DESCENT_ALTITUDE_THRESHOLD = 0.3   # meters; below this -> hand off to AUTO.LAND
MAX_HORIZONTAL_SPEED = 0.3
DESCENT_SPEED = 0.15
MARKER_TIMEOUT_SEC = 1.0           # marker considered "fresh" within this window
HOLD_LAST_CMD_SEC = 0.7            # after marker lost, hold last setpoint this long before stopping
SEARCH_TIMEOUT_SEC = 15.0
KP_XY = 0.5


class LandingControllerNode(Node):
    def __init__(self):
        super().__init__("landing_controller_node")
        self.get_logger().info("Landing Controller starting...")

        self.state = STATE_IDLE
        self.activated = False
        self.activation_time = None
        self.last_marker_time = None
        self.last_pose = None
        self._land_mode_sent = False
        self.last_cmd = Twist()

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            PoseStamped, "/precision_landing/marker_pose", self.marker_pose_cb, sensor_qos
        )
        self.create_subscription(State, "/mavros/state", self.state_cb, 10)
        self.create_subscription(Bool, "/precision_landing/activate", self.activate_cb, 10)
        self.current_mavros_state = None

        self.velocity_pub = self.create_publisher(
            Twist, "/mavros/setpoint_velocity/cmd_vel_unstamped", 10
        )
        self.status_pub = self.create_publisher(String, "/precision_landing/status", 10)

        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Landing Controller ready. Waiting for activation...")

    def state_cb(self, msg):
        self.current_mavros_state = msg

    def activate_cb(self, msg):
        if msg.data:
            self.get_logger().info("Activated by bridge node. Starting search for marker.")
            self.activated = True
            self.activation_time = time.time()
            self.last_marker_time = None
            self.last_pose = None
            self._land_mode_sent = False
            self.last_cmd = Twist()
            self.state = STATE_SEARCHING
        else:
            self.get_logger().info("Deactivated by bridge node.")
            self.activated = False
            self.state = STATE_IDLE
            self.velocity_pub.publish(Twist())

    def marker_pose_cb(self, msg: PoseStamped):
        self.last_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        self.last_marker_time = time.time()

        if self.activated and self.state == STATE_SEARCHING:
            self.get_logger().info("Marker acquired. Switching to ALIGNING.")
            self.state = STATE_ALIGNING

    def marker_is_fresh(self):
        if self.last_marker_time is None:
            return False
        return (time.time() - self.last_marker_time) < MARKER_TIMEOUT_SEC

    def control_loop(self):
        if not self.activated:
            return

        self.status_pub.publish(String(data=self.state))

        cmd = Twist()

        if self.state == STATE_SEARCHING:
            if self.last_marker_time is None and (time.time() - self.activation_time) > SEARCH_TIMEOUT_SEC:
                self.get_logger().error("No marker found within timeout. Reporting FAILED.")
                self.state = STATE_FAILED
                self.status_pub.publish(String(data=STATE_FAILED))
                self.activated = False
            return

        elif self.state == STATE_ALIGNING or self.state == STATE_DESCENDING:
            if not self.marker_is_fresh():
                time_since_lost = (
                    time.time() - self.last_marker_time if self.last_marker_time else 999.0
                )

                if time_since_lost < HOLD_LAST_CMD_SEC:
                    # Grace period: keep sending the LAST known command so the drone
                    # doesn't jerk to a stop or free-drift while waiting for marker to reappear
                    self.get_logger().warn(
                        f"Marker lost ({time_since_lost:.2f}s). Holding LAST setpoint."
                    )
                    self.velocity_pub.publish(self.last_cmd)
                else:
                    # Grace period expired, marker still not seen -> stop safely
                    self.get_logger().error(
                        "Marker lost beyond grace period. Stopping (zero velocity)."
                    )
                    self.velocity_pub.publish(Twist())
                    self.last_cmd = Twist()
                return

            x, y, z = self.last_pose
            vx = self._clamp(-KP_XY * x, -MAX_HORIZONTAL_SPEED, MAX_HORIZONTAL_SPEED)
            vy = self._clamp(-KP_XY * y, -MAX_HORIZONTAL_SPEED, MAX_HORIZONTAL_SPEED)
            aligned = abs(x) < XY_ALIGN_TOLERANCE and abs(y) < XY_ALIGN_TOLERANCE

            if self.state == STATE_ALIGNING:
                cmd.linear.x = vx
                cmd.linear.y = vy
                cmd.linear.z = 0.0

                if aligned:
                    self.get_logger().info("Aligned over marker. Switching to DESCENDING.")
                    self.state = STATE_DESCENDING

            elif self.state == STATE_DESCENDING:
                cmd.linear.x = vx * 0.5
                cmd.linear.y = vy * 0.5
                cmd.linear.z = -DESCENT_SPEED

                if z < DESCENT_ALTITUDE_THRESHOLD:
                    self.get_logger().info(
                        "Close enough to marker (Z < threshold). "
                        "Handing off to FCU AUTO.LAND — avoiding OFFBOARD near ground "
                        "to prevent ground-effect turbulence."
                    )
                    self.state = STATE_AUTO_LANDING

            self.last_cmd = cmd
            self.velocity_pub.publish(cmd)

        elif self.state == STATE_AUTO_LANDING:
            if not self._land_mode_sent:
                self._request_land_mode()

        elif self.state == STATE_LANDED:
            self.velocity_pub.publish(Twist())
            self.activated = False

        elif self.state == STATE_FAILED:
            self.velocity_pub.publish(Twist())
            self.activated = False

    def _request_land_mode(self):
        if not self.mode_client.service_is_ready():
            self.get_logger().warn("SetMode service not ready yet, retrying...")
            return

        req = SetMode.Request()
        req.custom_mode = "AUTO.LAND"
        future = self.mode_client.call_async(req)
        future.add_done_callback(self._land_mode_response_cb)
        self._land_mode_sent = True

    def _land_mode_response_cb(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("AUTO.LAND mode accepted by PX4.")
                self.state = STATE_LANDED
                self.status_pub.publish(String(data=STATE_LANDED))
            else:
                self.get_logger().error("AUTO.LAND mode REJECTED by PX4. Retrying...")
                self._land_mode_sent = False
        except Exception as e:
            self.get_logger().error(f"SetMode service call failed: {e}")
            self._land_mode_sent = False

    @staticmethod
    def _clamp(value, min_val, max_val):
        return max(min_val, min(max_val, value))


def main(args=None):
    rclpy.init(args=args)
    node = LandingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
