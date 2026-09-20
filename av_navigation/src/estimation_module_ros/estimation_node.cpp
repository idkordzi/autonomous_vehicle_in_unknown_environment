#include "estimation_node.hpp"


using std::placeholders::_1;


namespace NAVIGATION_ROS {


EstimationModuleNode::EstimationModuleNode(rclcpp::NodeOptions options)
: Node("av_estimation_node", options)
{
    this->declareRosParameters();
    this->initializeRosNodeConfig();
    this->initializeSubscribers();
    this->initializePublishers();
}


void EstimationModuleNode::ekfCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {

    geometry_msgs::msg::PoseStamped msg_pose = geometry_msgs::build<geometry_msgs::msg::PoseStamped>().header(
        msg->header
    ).pose(
        msg->pose.pose
    );

    geometry_msgs::msg::TwistStamped msg_velocity = geometry_msgs::build<geometry_msgs::msg::TwistStamped>().header(
        msg->header
    ).twist(
        msg->twist.twist
    );

    this->pub_robot_pose_->publish(msg_pose);
    this->pub_robot_velocity_->publish(msg_velocity);
}


void EstimationModuleNode::declareRosParameters() {

    this->declare_parameter("ros_node.subs.ekf", rclcpp::PARAMETER_STRING);

    this->declare_parameter("ros_node.pubs.robot_pose", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.pubs.robot_velocity", rclcpp::PARAMETER_STRING);
}


void EstimationModuleNode::initializeRosNodeConfig() {

    this->node_config_.sub_ekf = this->get_parameter("ros_node.subs.ekf").as_string();

    this->node_config_.pub_robot_pose = this->get_parameter("ros_node.pubs.robot_pose").as_string();
    this->node_config_.pub_robot_velocity = this->get_parameter("ros_node.pubs.robot_velocity").as_string();
}


void EstimationModuleNode::initializeSubscribers() {
    this->sub_ekf_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->node_config_.sub_ekf, 1, std::bind(&EstimationModuleNode::ekfCallback, this, _1));
}


void EstimationModuleNode::initializePublishers() {
    this->pub_robot_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(this->node_config_.pub_robot_pose, 1);
    this->pub_robot_velocity_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(this->node_config_.pub_robot_velocity, 1);
}


} // namespace NAVIGATION_ROS
