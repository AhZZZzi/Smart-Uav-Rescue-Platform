"""
GPS-to-VIO Switching Node — Final Robust Version
Listening to /mavros/state for real-time fallback triggers.
Fixed: VIO subscription QoS matched to default (was BEST_EFFORT-only,
causing mismatch with vio_node's default RELIABLE publisher, so
vio_cb was never firing).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import State
import time

GPS_FRESHNESS_TIMEOUT_SEC = 1.0
VIO_FRESHNESS_TIMEOUT_SEC = 5.0
MIN_TIME_BETWEEN_SWITCHES_SEC = 1.0

SOURCE_GPS = "GPS"
SOURCE_VIO = "VIO"
SOURCE_NONE = "NONE"


class GpsVioSwitchingNode(Node):
    def __init__(self):
        super().__init__('gps_vio_switching_node')

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscriptions
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_cb, sensor_qos
        )
        # NOTE: VIO node publishes with default (RELIABLE) QoS -- use default
        # QoS here too (depth=10), NOT BEST_EFFORT, to avoid QoS mismatch
        # silently dropping the connection.
        self.create_subscription(
            PoseStamped, '/vio/pose', self.vio_cb, 10
        )
        self.create_subscription(
            State, '/mavros/state', self.state_cb, sensor_qos
        )

        # Publishers
        self.vision_pose_pub = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose', 10
        )
        self.status_pub = self.create_publisher(String, '/localization/source', 10)

        self.current_gps = None
        self.last_gps_time = None
        self.last_vio_pose = None
        self.last_vio_time = None
        self.mavros_state = None

        self.current_source = SOURCE_GPS
        self.last_switch_time = time.time()

        self.create_timer(0.5, self.control_loop)
        self.get_logger().info("GPS-to-VIO Switching Node started (MAVROS State-Aware Mode)")

    def gps_cb(self, msg: NavSatFix):
        self.current_gps = msg
        self.last_gps_time = time.time()

    def vio_cb(self, msg: PoseStamped):
        self.last_vio_pose = msg
        self.last_vio_time = time.time()

    def state_cb(self, msg: State):
        self.mavros_state = msg

    def gps_is_reliable(self):
        if self.mavros_state is not None and not self.mavros_state.connected:
            return False

        if self.current_gps is None or self.last_gps_time is None:
            return False

        if (time.time() - self.last_gps_time) > GPS_FRESHNESS_TIMEOUT_SEC:
            return False

        return True

    def vio_is_fresh(self):
        if self.last_vio_time is None:
            return False
        return (time.time() - self.last_vio_time) < VIO_FRESHNESS_TIMEOUT_SEC

    def control_loop(self):
        gps_ok = self.gps_is_reliable()
        vio_ok = self.vio_is_fresh()

        # --- TEMPORARY TEST OVERRIDE: force GPS unreliable to verify switching logic
        # regardless of whether PX4's `failure gps off` actually stops NavSatFix output.
        # Set to False to restore normal GPS-based behavior. ---
        FORCE_GPS_FAIL_FOR_TEST = False
        if FORCE_GPS_FAIL_FOR_TEST:
            gps_ok = False

        self.get_logger().info(f"STATUS -> gps_ok: {gps_ok}, vio_ok: {vio_ok}, Active Source: {self.current_source}")

        desired_source = self.current_source
        time_since_switch = time.time() - self.last_switch_time
        can_switch = time_since_switch > MIN_TIME_BETWEEN_SWITCHES_SEC

        if not gps_ok and vio_ok and can_switch:
            desired_source = SOURCE_VIO
        elif gps_ok and self.current_source != SOURCE_GPS and can_switch:
            desired_source = SOURCE_GPS
        elif not gps_ok and not vio_ok:
            desired_source = SOURCE_NONE
            self.get_logger().error("CRITICAL: Both GPS and VIO are unreliable!")

        if desired_source != self.current_source:
            self.get_logger().warn(f"SWITCHING LOCALIZATION SOURCE: {self.current_source} -> {desired_source}")
            self.current_source = desired_source
            self.last_switch_time = time.time()

        self.status_pub.publish(String(data=self.current_source))

        if self.current_source == SOURCE_VIO and self.last_vio_pose is not None:
            out_msg = PoseStamped()
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = "vio_odom"
            out_msg.pose = self.last_vio_pose.pose
            self.vision_pose_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsVioSwitchingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()