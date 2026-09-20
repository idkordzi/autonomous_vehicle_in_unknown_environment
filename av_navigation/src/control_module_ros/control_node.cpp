#include "control_node.hpp"


using std::placeholders::_1;


namespace NAVIGATION_ROS {


ControlModuleNode::ControlModuleNode(rclcpp::NodeOptions options)
: Node("av_control_node", options)
{
    this->declareRosParameters();
    this->initializeRosNodeConfig();
    this->initializeComponents();
    this->initializeSubscribers();
    this->initializePublishers();
    this->initializeExecutionThread();
}


ControlModuleNode::~ControlModuleNode() {
    this->execute_worker_.join();
}


void ControlModuleNode::robotPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {

    std::lock_guard<std::mutex> lg(this->mtx_pose_);
    
    this->pose_cache_->header = msg->header;
    this->pose_cache_->pose = msg->pose;
}


void ControlModuleNode::robotVelocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg) {

    std::lock_guard<std::mutex> lg(this->mtx_velocity_);

    this->velocity_cache_->header = msg->header;
    this->velocity_cache_->twist = msg->twist;
}


void ControlModuleNode::goalPoseCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &msg) {

    std::lock_guard<std::mutex> lg(this->mtx_goal_);

    this->goal_cache_->header = msg->header;
    this->goal_cache_->vector = msg->vector;
}


void ControlModuleNode::executeThread() {
    while (rclcpp::ok()) {
        this->calculateControl();
        this->publish();
        if (this->execute_rate_)
            execute_rate_->sleep();
    }
}


void ControlModuleNode::calculateControl() {

    std::lock_guard<std::mutex> lg_pose(this->mtx_pose_);
    std::lock_guard<std::mutex> lg_velocity(this->mtx_velocity_);
    std::lock_guard<std::mutex> lg_goal(this->mtx_goal_);

    Eigen::Vector3f robot_position = Eigen::Vector3f(
        this->pose_cache_->pose.position.x,
        this->pose_cache_->pose.position.y,
        this->pose_cache_->pose.position.z
    );
    Eigen::Vector4f robot_orientation = Eigen::Vector4f(
        this->pose_cache_->pose.orientation.x,
        this->pose_cache_->pose.orientation.y,
        this->pose_cache_->pose.orientation.z,
        this->pose_cache_->pose.orientation.w
    );
    Eigen::Vector3f robot_velocity = Eigen::Vector3f(
        this->velocity_cache_->twist.linear.x,
        this->velocity_cache_->twist.linear.y,
        this->velocity_cache_->twist.linear.z
    );
    Eigen::Vector3f robot_rotation = Eigen::Vector3f(
        this->velocity_cache_->twist.angular.x,
        this->velocity_cache_->twist.angular.y,
        this->velocity_cache_->twist.angular.z
    );
    Eigen::Vector3f goal_position = Eigen::Vector3f(
        this->goal_cache_->vector.x,
        this->goal_cache_->vector.y,
        this->goal_cache_->vector.z
    );

    this->controller_->setGoal(goal_position);
    std::vector<float> control = this->controller_->calculateControl(
        robot_position,
        robot_orientation,
        robot_velocity,
        robot_rotation
    );

    this->msg_velocity_->twist.linear.x = control[0];
    this->msg_velocity_->twist.angular.z = control[1];
}


void ControlModuleNode::publish() {

    this->msg_velocity_->header.stamp = this->get_clock()->now();
    this->msg_velocity_->header.frame_id = "odom";
    this->pub_robot_velocity_->publish(*this->msg_velocity_);
}


void ControlModuleNode::declareRosParameters() {

    this->declare_parameter("ros_node.subs.robot_pose", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.subs.robot_velocity", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.subs.goal_pose", rclcpp::PARAMETER_STRING);

    this->declare_parameter("ros_node.pubs.robot_velocity", rclcpp::PARAMETER_STRING);

    this->declare_parameter("ros_node.thread_freq", rclcpp::PARAMETER_DOUBLE);

    this->declare_parameter("controller.reg_forward", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("controller.reg_forward_max", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("controller.reg_rotate", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("controller.reg_rotate_max", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("controller.max_pos_err_ang_off", rclcpp::PARAMETER_DOUBLE);
}


void ControlModuleNode::initializeRosNodeConfig() {

    this->node_config_.sub_robot_pose = this->get_parameter("ros_node.subs.robot_pose").as_string();
    this->node_config_.sub_robot_velocity = this->get_parameter("ros_node.subs.robot_velocity").as_string();
    this->node_config_.sub_goal_pose = this->get_parameter("ros_node.subs.goal_pose").as_string();

    this->node_config_.pub_robot_velocity = this->get_parameter("ros_node.pubs.robot_velocity").as_string();

    this->node_config_.thread_freq = (float)(this->get_parameter("ros_node.thread_freq").as_double());
}


void ControlModuleNode::initializeComponents() {
  
    this->controller_config_.reg_forward_max_abs = this->get_parameter("controller.reg_forward_max").as_double();
    this->controller_config_.reg_rotate_max_abs = this->get_parameter("controller.reg_rotate_max").as_double();
    this->controller_config_.max_pos_err_ang_off = this->get_parameter("controller.max_pos_err_ang_off").as_double();
    this->controller_config_.execution_time = 1.0f / this->node_config_.thread_freq;

    std::vector<double> reg_forward = this->get_parameter("controller.reg_forward").as_double_array();
    std::vector<double> reg_rotate = this->get_parameter("controller.reg_rotate").as_double_array();

    this->controller_config_.reg_forward_Kp = reg_forward[0];
    this->controller_config_.reg_forward_Ki = reg_forward[1];
    this->controller_config_.reg_forward_Kd = reg_forward[2];

    this->controller_config_.reg_rotate_Kp = reg_rotate[0];
    this->controller_config_.reg_rotate_Ki = reg_rotate[1];
    this->controller_config_.reg_rotate_Kd = reg_rotate[2];

    this->controller_ = std::make_unique<NAVIGATION_CORE::Controller>(this->controller_config_);
    this->controller_->reset();
}


void ControlModuleNode::initializeSubscribers() {

    this->sub_robot_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->node_config_.sub_robot_pose, 1, std::bind(&ControlModuleNode::robotPoseCallback, this, _1));
    this->sub_robot_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        this->node_config_.sub_robot_velocity, 1, std::bind(&ControlModuleNode::robotVelocityCallback, this, _1));
    this->sub_goal_pose_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        this->node_config_.sub_goal_pose, 1, std::bind(&ControlModuleNode::goalPoseCallback, this, _1));

    this->pose_cache_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    this->velocity_cache_ = std::make_shared<geometry_msgs::msg::TwistStamped>();
    this->goal_cache_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();

    this->pose_cache_->pose.position.x = 0.0f;
    this->pose_cache_->pose.position.y = 0.0f;
    this->pose_cache_->pose.position.z = 0.0f;
    this->pose_cache_->pose.orientation.x = 0.0f;
    this->pose_cache_->pose.orientation.y = 0.0f;
    this->pose_cache_->pose.orientation.z = 0.0f;
    this->pose_cache_->pose.orientation.w = 1.0f;

    this->velocity_cache_->twist.linear.x = 0.0f;
    this->velocity_cache_->twist.linear.y = 0.0f;
    this->velocity_cache_->twist.linear.z = 0.0f;
    this->velocity_cache_->twist.angular.x = 0.0f;
    this->velocity_cache_->twist.angular.y = 0.0f;
    this->velocity_cache_->twist.angular.z = 0.0f;

    this->goal_cache_->vector.x = 0.0f;
    this->goal_cache_->vector.y = 0.0f;
    this->goal_cache_->vector.z = 0.0f;
}


void ControlModuleNode::initializePublishers() {
    this->pub_robot_velocity_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(this->node_config_.pub_robot_velocity, 1);
    this->msg_velocity_ = std::make_shared<geometry_msgs::msg::TwistStamped>();

    this->msg_velocity_->twist.linear.x = 0.0f;
    this->msg_velocity_->twist.linear.y = 0.0f;
    this->msg_velocity_->twist.linear.z = 0.0f;
    this->msg_velocity_->twist.angular.x = 0.0f;
    this->msg_velocity_->twist.angular.y = 0.0f;
    this->msg_velocity_->twist.angular.z = 0.0f;
}


void ControlModuleNode::initializeExecutionThread() {
    this->execute_rate_ = std::make_unique<rclcpp::Rate>(this->node_config_.thread_freq);
    this->execute_worker_ = std::thread(&ControlModuleNode::executeThread, this);
}


} // namespace NAVIGATION_ROS
