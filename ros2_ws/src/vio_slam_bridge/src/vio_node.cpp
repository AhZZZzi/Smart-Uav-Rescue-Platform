/**
 * VIO/SLAM ROS2 Node — wraps ORB-SLAM3 monocular tracking.
 * Subscribes to /camera/image_raw (published by camera_reader_node) instead
 * of opening the camera device directly, so multiple nodes can share
 * the same physical camera without device conflicts.
 * Publishes camera pose to /vio/pose.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <System.h>
#include <chrono>
#include <cstring>

class VioSlamNode : public rclcpp::Node
{
public:
    VioSlamNode(const std::string &vocabPath, const std::string &settingsPath)
        : Node("vio_slam_node"), SLAM(vocabPath, settingsPath, ORB_SLAM3::System::MONOCULAR, false)
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vio/pose", 10);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&VioSlamNode::imageCallback, this, std::placeholders::_1));

        t0_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(this->get_logger(), "VIO/SLAM node started. Subscribed to /camera/image_raw, publishing to /vio/pose");
    }

    ~VioSlamNode()
    {
        SLAM.Shutdown();
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Manual conversion from sensor_msgs/Image (bgr8) to cv::Mat
        // (no cv_bridge dependency, to avoid NumPy/OpenCV version conflicts)
        cv::Mat frame(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(msg->data.data()), msg->step);
        cv::Mat frame_copy = frame.clone();  // own the memory before ORB-SLAM3 processes it

        double tframe = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now() - t0_).count();

        Sophus::SE3f Tcw = SLAM.TrackMonocular(frame_copy, tframe);

        int state = SLAM.GetTrackingState();
        if (state != 2 /* OK */)
        {
            return;
        }

        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Vector3f t = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "vio_odom";
        pose_msg.pose.position.x = t.x();
        pose_msg.pose.position.y = t.y();
        pose_msg.pose.position.z = t.z();
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pose_pub_->publish(pose_msg);
    }

    ORB_SLAM3::System SLAM;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::chrono::steady_clock::time_point t0_;
};

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: ros2 run vio_slam_bridge vio_node <path_to_vocabulary> <path_to_settings>" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<VioSlamNode>(argv[1], argv[2]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
