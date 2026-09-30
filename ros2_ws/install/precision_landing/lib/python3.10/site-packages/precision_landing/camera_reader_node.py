"""
Camera Reader Node — single source of truth for the webcam.
Publishes raw frames to /camera/image_raw so multiple downstream
nodes (ArUco detector, VIO/SLAM, obstacle detection) can share
the same physical camera without device conflicts.
Manual OpenCV <-> ROS2 Image conversion (no cv_bridge dependency,
to avoid NumPy/OpenCV version conflicts with the system cv_bridge build).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 10


def cv2_to_image_msg(frame, frame_id="camera_link"):
    """Manually convert a BGR OpenCV frame to sensor_msgs/Image (no cv_bridge)."""
    msg = Image()
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = frame.shape[1] * 3
    msg.data = frame.tobytes()
    msg.header.frame_id = frame_id
    return msg


class CameraReaderNode(Node):
    def __init__(self):
        super().__init__('camera_reader_node')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        self.cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera /dev/video{CAMERA_INDEX}")
            return

        self.timer = self.create_timer(1.0 / FPS, self.publish_frame)
        self.get_logger().info(f"Camera reader started. Publishing to /camera/image_raw at {FPS} Hz")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Empty frame, skipping.")
            return

        frame = np.ascontiguousarray(frame, dtype=np.uint8)
        msg = cv2_to_image_msg(frame)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'cap'):
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()