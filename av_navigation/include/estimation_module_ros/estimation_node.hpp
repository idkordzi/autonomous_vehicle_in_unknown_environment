#pragma once

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"


namespace NAVIGATION_ROS {


struct EstimationModuleNodeConfig {
    std::string sub_ekf = "/odometry/filtered";
    std::string pub_robot_pose = "/robot/ekf/pose";
    std::string pub_robot_velocity = "/robot/ekf/velocity";
};


class EstimationModuleNode : public rclcpp::Node {

public:
    EstimationModuleNode(rclcpp::NodeOptions options);
    ~EstimationModuleNode() = default;

private:

    //initialization
    void declareRosParameters();
    void initializeRosNodeConfig();
    void initializeSubscribers();
    void initializePublishers();

    // callbacks
    void ekfCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);

    EstimationModuleNodeConfig node_config_ = {};

    // subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ekf_;

    // publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_robot_pose_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_robot_velocity_;
};


} // namespace NAVIGATION_ROS
