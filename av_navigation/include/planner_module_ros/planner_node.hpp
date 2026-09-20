#pragma once

#include <string>
#include <mutex>
#include <thread>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "local_planner.hpp"


namespace NAVIGATION_ROS {


struct PlannerModuleNodeConfig {
    std::string sub_vision_cloud = "/robot/vision/cloud";
    std::string sub_vision_target = "/robot/vision/target";
    std::string sub_robot_pose = "/robot/ekf/pose";
    std::string sub_robot_velocity = "/robot/ekf/velocity";

    std::string pub_planner_goal = "/robot/planner/goal";
    std::string pub_planner_image_histogram = "/robot/planner/image/histogram";
    std::string pub_planner_image_cost = "/robot/planner/image/cost";

    float thread_freq = 50.0f;
};


class PlannerModuleNode : public rclcpp::Node {

public:
    PlannerModuleNode(rclcpp::NodeOptions options);
    ~PlannerModuleNode();

private:

    // initialization
    void declareRosParameters();
    void initializeRosNodeConfig();
    void initializeComponents();
    void initializeSubscribers();
    void initializePublishers();
    void initializeClassMembers();
    void initializeExecutionThread();

    // callbacks
    void visionCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    void visionTargetCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &msg);
    void robotPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
    void robotVelocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);

    void executeThread();
    void updatePlanner();
    void runPlanner();
    void getPlannerDebug();
    void publish();

    PlannerModuleNodeConfig node_config_ = {};
    NAVIGATION_CORE::LocalPlannerConfig planner_config_ = {};
    std::unique_ptr<NAVIGATION_CORE::LocalPlanner> planner_;

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_vision_cloud_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_vision_target_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_robot_pose_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_robot_velocity_;

    // subscription msg cache
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_cache_;
    geometry_msgs::msg::Vector3Stamped::SharedPtr target_cache_;
    geometry_msgs::msg::PoseStamped::SharedPtr pose_cache_;
    geometry_msgs::msg::TwistStamped::SharedPtr velocity_cache_;

    // publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_planner_goal_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_planner_img_histogram_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_planner_img_cost_;

    // publishers msg cache
    geometry_msgs::msg::Vector3Stamped::SharedPtr msg_goal_;
    sensor_msgs::msg::Image::SharedPtr msg_img_histogram_;
    sensor_msgs::msg::Image::SharedPtr msg_img_cost_;

    // execution thread
    std::unique_ptr<rclcpp::Rate> execute_rate_;
    std::thread execute_worker_;

    std::mutex mtx_cloud_ = {};
    std::mutex mtx_target_ = {};
    std::mutex mtx_pose_ = {};
    std::mutex mtx_velocity_ = {};

    bool cloud_ready_ = false;
    bool target_ready_ = false;
    bool pose_ready_ = false;
    bool velocity_ready_ = false;

    Eigen::Vector3f current_goal_ = {};
};


} // namespace NAVIGATION_ROS
