#pragma once

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <memory>

#include "eigen3/Eigen/Dense"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "controller.hpp"


namespace NAVIGATION_ROS {

struct ControlModuleNodeConfig {

    std::string sub_robot_pose = "/robot/ekf/pose";
    std::string sub_robot_velocity = "/robot/ekf/velocity";
    std::string sub_goal_pose = "/robot/planner/goal";

    std::string pub_robot_velocity = "/robot/control/velocity";

    float thread_freq = 100.0f; // [Hz]
};

class ControlModuleNode : public rclcpp::Node {

public:
    ControlModuleNode(rclcpp::NodeOptions options);
    ~ControlModuleNode();

private:

    // initialization
    void declareRosParameters();
    void initializeRosNodeConfig();
    void initializeComponents();
    void initializeSubscribers();
    void initializePublishers();
    void initializeExecutionThread();

    // callbacks
    void robotPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void robotVelocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg);
    void goalPoseCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &msg);

    // execution thread
    void executeThread();
    void calculateControl();
    void publish();

    ControlModuleNodeConfig node_config_ = {};
    NAVIGATION_CORE::ControllerConfig controller_config_ = {};
    std::unique_ptr<NAVIGATION_CORE::Controller> controller_;

    // subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_robot_pose_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_robot_velocity_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_goal_pose_;

    // subscription msg cache
    geometry_msgs::msg::PoseStamped::SharedPtr pose_cache_;
    geometry_msgs::msg::TwistStamped::SharedPtr velocity_cache_;
    geometry_msgs::msg::Vector3Stamped::SharedPtr goal_cache_;

    // publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_robot_velocity_;

    // published msg cache
    geometry_msgs::msg::TwistStamped::SharedPtr msg_velocity_;

    // execution thread
    std::unique_ptr<rclcpp::Rate> execute_rate_;
    std::thread execute_worker_;

    std::mutex mtx_pose_ = {};
    std::mutex mtx_velocity_ = {};
    std::mutex mtx_goal_ = {};
};


} // namespace NAVIGATION_ROS
