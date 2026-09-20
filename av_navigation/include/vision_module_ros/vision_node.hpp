#pragma once

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <memory>

#include "eigen3/Eigen/Dense"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "tracker.hpp"


namespace NAVIGATION_ROS {


struct VisionModuleNodeConfig {
    std::string sub_camera_color = "/robot/sensors/camera/color";
    std::string sub_camera_depth = "/robot/sensors/camera/depth";
    std::string sub_camera_cloud = "/robot/sensors/camera/cloud";
    std::string sub_robot_pose = "/robot/ekf/pose";

    std::string pub_vision_target = "/robot/vision/target";
    std::string pub_vision_cloud = "/robot/vision/cloud";
    std::string pub_vision_goal = "/robot/vision/goal";

    float thread_freq = 30.0f; // [Hz]
    bool follow_mode = false;
};


class VisionModuleNode : public rclcpp::Node {

public:
    VisionModuleNode(rclcpp::NodeOptions options);
    ~VisionModuleNode();

private:

    //initialization
    void declareRosParameters();
    void initializeRosNodeConfig();
    void initializeComponents();
    void initializeSubscribers();
    void initializePublishers();
    void initializeExecutionThread();

    // callbacks
    void cameraColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void cameraDepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void cameraCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    void robotPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);

    void executeThread();
    void processInputData();
    void detectTarget();
    void follow_target();
    void publish();

    void cloudToLocalStorage();
    void cloudFromLocalStorage();
    void convertCloud();

    VisionModuleNodeConfig node_config_ = {};
    NAVIGATION_CORE::TrackerConfig tracker_config_ = {};
    std::unique_ptr<NAVIGATION_CORE::Tracker> tracker_;

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_color_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_depth_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_camera_cloud_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_robot_pose_;

    // subscription msg cache
    sensor_msgs::msg::Image::SharedPtr img_color_cache_;
    sensor_msgs::msg::Image::SharedPtr img_depth_cache_;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_cache_;
    geometry_msgs::msg::PoseStamped::SharedPtr pose_cache_;

    // publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_vision_target_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_vision_cloud_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_vision_goal_;

    // published msg cache
    geometry_msgs::msg::Vector3Stamped::SharedPtr msg_target_;
    sensor_msgs::msg::PointCloud2::SharedPtr msg_cloud_;
    geometry_msgs::msg::Vector3Stamped::SharedPtr msg_goal_;

    // execution thread
    std::unique_ptr<rclcpp::Rate> execute_rate_;
    std::thread execute_worker_;

    std::mutex mtx_img_color_ = {};
    std::mutex mtx_img_depth_ = {};
    std::mutex mtx_cloud_ = {};
    std::mutex mtx_pose_ = {};

    bool img_color_ready_ = false;
    bool img_depth_ready_ = false;
    bool cloud_ready_ = false;
    bool pose_ready_ = false;

    cv_bridge::CvImagePtr cv_img_cache_ = {};

    cv::Mat color_frame_ = {};
    cv::Mat depth_frame_ = {};

    unsigned cloud_size_ = 0;
    std::vector<float> cloud_flattened_ = {};

    Eigen::Vector3f current_position_ = {};
    Eigen::Vector4f current_orientation_ = {};

    bool target_found_ = false;
};


} // namespace NAVIGATION_ROS
