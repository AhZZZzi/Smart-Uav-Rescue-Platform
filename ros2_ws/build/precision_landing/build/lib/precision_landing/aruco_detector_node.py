"""
ArUco Detector Node — subscribes to /camera/image_raw (published by
camera_reader_node) instead of opening the camera device directly.
This allows multiple nodes (ArUco, VIO/SLAM, obstacle detection) to
share a single physical camera without device conflicts.
No cv_bridge dependency (manual message conversion) to avoid
NumPy/OpenCV version conflicts with the system cv_bridge build.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import numpy as np
import os

# --- CONFIGURATION ---
MARKER_SIZE_M = 0.15
ARUCO_DICT = aruco.DICT_4X4_50

CALIBRATION_FILE_PATH = "/home/huyen/drone_project/standalone_tests/precision_landing_prototype/camera_calibration.npz"


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        self.pose_pub = self.create_publisher(PoseStamped, '/precision_landing/marker_pose', 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

        if os.path.exists(CALIBRATION_FILE_PATH):
            calib_data = np.load(CALIBRATION_FILE_PATH)
            self.camera_matrix = calib_data["camera_matrix"]
            self.dist_coeffs = calib_data["dist_coeffs"]
            self.get_logger().info(f"Loaded REAL calibration from {CALIBRATION_FILE_PATH}")
        else:
            self.get_logger().warn(
                f"Calibration file NOT FOUND at {CALIBRATION_FILE_PATH}. Using rough fallback."
            )
            self.camera_matrix = np.array([
                [800.0, 0.0, 320.0],
                [0.0, 800.0, 240.0],
                [0.0, 0.0, 1.0]
            ], dtype=float)
            self.dist_coeffs = np.zeros((4, 1))

        half_size = MARKER_SIZE_M / 2
        self.marker_points_3d = np.array([
            [-half_size,  half_size, 0],
            [ half_size,  half_size, 0],
            [ half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

        self.get_logger().info("ArUco Detector Node started. Subscribed to /camera/image_raw")

    def image_callback(self, msg: Image):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            success, rvec, tvec = cv2.solvePnP(
                self.marker_points_3d, corners[0], self.camera_matrix, self.dist_coeffs
            )

            if not success:
                return

            out_msg = PoseStamped()
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = "camera_link"
            out_msg.pose.position.x = float(tvec[0][0])
            out_msg.pose.position.y = float(tvec[1][0])
            out_msg.pose.position.z = float(tvec[2][0])

            self.pose_pub.publish(out_msg)
            self.get_logger().info(
                f"Marker found! [X: {out_msg.pose.position.x:.2f}, "
                f"Y: {out_msg.pose.position.y:.2f}, Z: {out_msg.pose.position.z:.2f}]"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()