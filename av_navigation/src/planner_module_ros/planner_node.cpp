#include "planner_node.hpp"


using std::placeholders::_1;


namespace NAVIGATION_ROS {


PlannerModuleNode::PlannerModuleNode(rclcpp::NodeOptions options)
: Node("av_planner_node", options)
{
    this->declareRosParameters();
    this->initializeRosNodeConfig();
    this->initializeComponents();
    this->initializeSubscribers();
    this->initializePublishers();
    this->initializeExecutionThread();
}


PlannerModuleNode::~PlannerModuleNode() {
    this->execute_worker_.join();
}


void PlannerModuleNode::visionCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {

    std::lock_guard<std::mutex> lg(this->mtx_cloud_);

    this->cloud_cache_->header = msg->header;

    // meta data
    this->cloud_cache_->width        = msg->width;
    this->cloud_cache_->height       = msg->height;
    this->cloud_cache_->point_step   = msg->point_step;
    this->cloud_cache_->row_step     = msg->row_step;
    this->cloud_cache_->is_bigendian = msg->is_bigendian;
    this->cloud_cache_->is_dense     = msg->is_dense;
    this->cloud_cache_->fields       = msg->fields;

    // point data
    this->cloud_cache_->data = msg->data;

    this->cloud_ready_ = true;
}


void PlannerModuleNode::visionTargetCallback(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr &msg) {

    std::lock_guard<std::mutex> lg(this->mtx_target_);

    this->target_cache_->header = msg->header;
    this->target_cache_->vector = msg->vector;

    this->target_ready_ = true;
}


void PlannerModuleNode::robotPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {

    std::lock_guard<std::mutex> lg(this->mtx_pose_);

    this->pose_cache_->header = msg->header;
    this->pose_cache_->pose = msg->pose;

    this->pose_ready_ = true;
}


void PlannerModuleNode::robotVelocityCallback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg) {

    std::lock_guard<std::mutex> lg(this->mtx_velocity_);

    this->velocity_cache_->header = msg->header;
    this->velocity_cache_->twist = msg->twist;

    this->velocity_ready_ = true;
}


void PlannerModuleNode::executeThread() {
  
    while (rclcpp::ok()) {
        if (this->pose_ready_ && this->velocity_ready_) {
            this->updatePlanner();
            this->runPlanner();
            this->getPlannerDebug();
            this->publish();
        }
        if (execute_rate_)
            execute_rate_->sleep();
    }
}


void PlannerModuleNode::updatePlanner() {

    std::lock_guard<std::mutex> lg_target(this->mtx_target_);
    std::lock_guard<std::mutex> lg_cloud(this->mtx_cloud_);
    std::lock_guard<std::mutex> lg_pose(this->mtx_pose_);
    std::lock_guard<std::mutex> lg_velocity(this->mtx_velocity_);   

    Eigen::Vector3f position = Eigen::Vector3f(
        this->pose_cache_->pose.position.x,
        this->pose_cache_->pose.position.y,
        this->pose_cache_->pose.position.z
    );
    Eigen::Vector4f orientation = Eigen::Vector4f(
        this->pose_cache_->pose.orientation.x,
        this->pose_cache_->pose.orientation.y,
        this->pose_cache_->pose.orientation.z,
        this->pose_cache_->pose.orientation.w
    );
    Eigen::Vector3f velocity = Eigen::Vector3f(
        this->velocity_cache_->twist.linear.x,
        this->velocity_cache_->twist.linear.y,
        this->velocity_cache_->twist.linear.z
    );
    this->planner_->setState(position, orientation, velocity);

    if (this->target_ready_) {
        Eigen::Vector3f target = Eigen::Vector3f(
            this->target_cache_->vector.x,
            this->target_cache_->vector.y,
            this->target_cache_->vector.z
        );
        this->planner_->setTarget(target);
    }

    if (this->cloud_ready_) {
        NAVIGATION_CORE::PointCloud<NAVIGATION_CORE::PointXYZ> new_cloud = NAVIGATION_CORE::PointCloud<NAVIGATION_CORE::PointXYZ>();
        sensor_msgs::PointCloud2Iterator<float> iter_x(*(this->cloud_cache_), "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*(this->cloud_cache_), "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*(this->cloud_cache_), "z");
        
        for (unsigned i = 0; i < this->cloud_cache_->height*this->cloud_cache_->width; i++, iter_x += 1, iter_y += 1, iter_z += 1) {
            new_cloud.push_back(NAVIGATION_CORE::PointXYZ(*(iter_x), *(iter_y), *(iter_z)));
        }
        this->planner_->setPointCloud(new_cloud);
    }

    this->pose_ready_ = false;
    this->velocity_ready_ = false;
    this->target_ready_ = false;
    this->cloud_ready_ = false;
}


void PlannerModuleNode::runPlanner() {
    this->planner_->run();
    this->current_goal_ = this->planner_->getNext();
}


void PlannerModuleNode::getPlannerDebug() {

    cv::Mat hist_image = this->planner_->getHistogramImage();
    this->msg_img_histogram_ = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", hist_image).toImageMsg();

    cv::Mat cost_image = this->planner_->getCostImage();
    this->msg_img_cost_ = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", cost_image).toImageMsg();
}


void PlannerModuleNode::publish() {

    this->msg_goal_->header.stamp = this->get_clock()->now();
    this->msg_goal_->header.frame_id = "odom";
    this->pub_planner_goal_->publish(*(this->msg_goal_));

    this->msg_img_histogram_->header.stamp = this->get_clock()->now();
    this->pub_planner_img_histogram_->publish(*(this->msg_img_histogram_));

    this->msg_img_cost_->header.stamp = this->get_clock()->now();
    this->pub_planner_img_cost_->publish(*(this->msg_img_cost_));
}


void PlannerModuleNode::declareRosParameters() {

    this->declare_parameter("ros_node.subs.vision_cloud", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.subs.vision_target", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.subs.robot_pose", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.subs.robot_velocity", rclcpp::PARAMETER_STRING);

    this->declare_parameter("ros_node.pubs.planner_goal", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.pubs.planner_image_histogram", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.pubs.planner_image_cost", rclcpp::PARAMETER_STRING);

    this->declare_parameter("ros_node.thread_freq", rclcpp::PARAMETER_DOUBLE);

    this->declare_parameter("planner.enable_cuda", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("planner.skip_planning", rclcpp::PARAMETER_BOOL);

    this->declare_parameter("planner.sensor_range_min", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("planner.sensor_range_max", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("planner.camera_fov_h", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("planner.camera_fov_v", rclcpp::PARAMETER_DOUBLE);

    this->declare_parameter("planner.alpha", rclcpp::PARAMETER_INTEGER);

    this->declare_parameter("planner.point_max_age", rclcpp::PARAMETER_DOUBLE);

    this->declare_parameter("planner.goal_dev_margin", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("planner.goal_min_dist", rclcpp::PARAMETER_DOUBLE);

    this->declare_parameter("planner.max_candidates_per_it", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("planner.robot_pos_margin", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("planner.planning_step", rclcpp::PARAMETER_DOUBLE);

    this->declare_parameter("planner.cost_yaw", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("planner.cost_velocity", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("planner.cost_obstacle_distance", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("planner.obstacle_distance_min", rclcpp::PARAMETER_DOUBLE);
    
}


void PlannerModuleNode::initializeRosNodeConfig() {

    this->node_config_.sub_vision_cloud   = this->get_parameter("ros_node.subs.vision_cloud").as_string();
    this->node_config_.sub_vision_target  = this->get_parameter("ros_node.subs.vision_target").as_string();
    this->node_config_.sub_robot_pose     = this->get_parameter("ros_node.subs.robot_pose").as_string();
    this->node_config_.sub_robot_velocity = this->get_parameter("ros_node.subs.robot_velocity").as_string();

    this->node_config_.pub_planner_goal = this->get_parameter("ros_node.pubs.planner_goal").as_string();
    this->node_config_.pub_planner_image_histogram = this->get_parameter("ros_node.pubs.planner_image_histogram").as_string();
    this->node_config_.pub_planner_image_cost = this->get_parameter("ros_node.pubs.planner_image_cost").as_string();

    this->node_config_.thread_freq = (float)(this->get_parameter("ros_node.thread_freq").as_double());
}


void PlannerModuleNode::initializeComponents() {

    this->planner_config_.enable_cuda = this->get_parameter("planner.enable_cuda").as_bool();
    this->planner_config_.skip_planning = this->get_parameter("planner.skip_planning").as_bool();
    this->planner_config_.execution_time = 1.0f / this->node_config_.thread_freq;

    this->planner_config_.sensor_range_min = (float)(this->get_parameter("planner.sensor_range_min").as_double());
    this->planner_config_.sensor_range_max = (float)(this->get_parameter("planner.sensor_range_max").as_double());
    this->planner_config_.camera_fov_h = (float)(this->get_parameter("planner.camera_fov_h").as_double());
    this->planner_config_.camera_fov_v = (float)(this->get_parameter("planner.camera_fov_v").as_double());

    this->planner_config_.alpha = (float)(this->get_parameter("planner.alpha").as_int());

    this->planner_config_.point_max_age   = this->get_parameter("planner.point_max_age").as_double();

    this->planner_config_.goal_dev_margin = (float)(this->get_parameter("planner.goal_dev_margin").as_double());
    this->planner_config_.goal_min_dist     = (float)(this->get_parameter("planner.goal_min_dist").as_double());


    this->planner_config_.max_candidates_per_it = this->get_parameter("planner.max_candidates_per_it").as_int();
    this->planner_config_.robot_pos_margin = (float)(this->get_parameter("planner.robot_pos_margin").as_double());
    this->planner_config_.planning_step = (float)(this->get_parameter("planner.planning_step").as_double());

    this->planner_config_.cost_yaw = (float)(this->get_parameter("planner.cost_yaw").as_double());
    this->planner_config_.cost_velocity = (float)(this->get_parameter("planner.cost_velocity").as_double());
    this->planner_config_.cost_obstacle_distance = (float)(this->get_parameter("planner.cost_obstacle_distance").as_double());
    this->planner_config_.obstacle_distance_min = (float)(this->get_parameter("planner.obstacle_distance_min").as_double());

    this->planner_ = std::make_unique<NAVIGATION_CORE::LocalPlanner>(this->planner_config_);
    this->current_goal_ = Eigen::Vector3f::Zero();
}


void PlannerModuleNode::initializeSubscribers() {

    this->sub_vision_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        this->node_config_.sub_vision_cloud, 1, std::bind(&PlannerModuleNode::visionCloudCallback, this, _1));
    this->sub_vision_target_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        this->node_config_.sub_vision_target, 1, std::bind(&PlannerModuleNode::visionTargetCallback, this, _1));
    this->sub_robot_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->node_config_.sub_robot_pose, 1, std::bind(&PlannerModuleNode::robotPoseCallback, this, _1));
    this->sub_robot_velocity_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        this->node_config_.sub_robot_velocity, 1, std::bind(&PlannerModuleNode::robotVelocityCallback, this, _1));

    this->cloud_cache_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    this->target_cache_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
    this->pose_cache_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
    this->velocity_cache_ = std::make_shared<geometry_msgs::msg::TwistStamped>();
}


void PlannerModuleNode::initializePublishers() {

    this->pub_planner_goal_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(this->node_config_.pub_planner_goal, 1);
    this->pub_planner_img_histogram_ = this->create_publisher<sensor_msgs::msg::Image>(this->node_config_.pub_planner_image_histogram, 1);
    this->pub_planner_img_cost_ = this->create_publisher<sensor_msgs::msg::Image>(this->node_config_.pub_planner_image_cost, 1);

    this->msg_goal_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
    this->msg_img_histogram_ = std::make_shared<sensor_msgs::msg::Image>();
    this->msg_img_cost_ = std::make_shared<sensor_msgs::msg::Image>();
}


void PlannerModuleNode::initializeExecutionThread() {
    this->execute_rate_   = std::make_unique<rclcpp::Rate>(this->node_config_.thread_freq);
    this->execute_worker_ = std::thread(&PlannerModuleNode::executeThread, this);
}

} // namespace NAVIGATION_ROS
