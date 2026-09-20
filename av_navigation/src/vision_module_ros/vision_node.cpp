#include "vision_node.hpp"


using std::placeholders::_1;


namespace NAVIGATION_ROS {


VisionModuleNode::VisionModuleNode(rclcpp::NodeOptions options)
: Node("av_vision_node", options)
{
    this->declareRosParameters();
    this->initializeRosNodeConfig();
    this->initializeComponents();
    this->initializeSubscribers();
    this->initializePublishers();
    this->initializeExecutionThread();
}


VisionModuleNode::~VisionModuleNode() {
    this->execute_worker_.join();
}


void VisionModuleNode::cameraColorCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {

    std::lock_guard<std::mutex> lg(this->mtx_img_color_);

    this->img_color_cache_->header       = msg->header;
    this->img_color_cache_->height       = msg->height;
    this->img_color_cache_->width        = msg->width;
    this->img_color_cache_->encoding     = msg->encoding;
    this->img_color_cache_->is_bigendian = msg->is_bigendian;
    this->img_color_cache_->step         = msg->step;
    this->img_color_cache_->data         = msg->data;

    this->img_color_ready_ = true;
}


void VisionModuleNode::cameraDepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {

    std::lock_guard<std::mutex> lg(this->mtx_img_depth_);

    this->img_depth_cache_->header       = msg->header;
    this->img_depth_cache_->height       = msg->height;
    this->img_depth_cache_->width        = msg->width;
    this->img_depth_cache_->encoding     = msg->encoding;
    this->img_depth_cache_->is_bigendian = msg->is_bigendian;
    this->img_depth_cache_->step         = msg->step;
    this->img_depth_cache_->data         = msg->data;

    this->img_depth_ready_ = true;
}


void VisionModuleNode::cameraCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {

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

    // point cloud data
    this->cloud_cache_->data = msg->data;

    this->cloud_ready_ = true;
}


void VisionModuleNode::robotPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {

    std::lock_guard<std::mutex> lg(this->mtx_pose_);

    this->pose_cache_->header = msg->header;
    this->pose_cache_->pose = msg->pose;

    this->pose_ready_ = true;
}


void VisionModuleNode::executeThread() {

    while (rclcpp::ok()) {
        if (this->img_color_ready_ && this->cloud_ready_ && this->pose_ready_) {
            this->processInputData();
            this->detectTarget();
            if (this->node_config_.follow_mode) this->follow_target();
            this->publish();
        }
        if (execute_rate_)
            execute_rate_->sleep();
    }
}


void VisionModuleNode::processInputData() {
    std::lock_guard<std::mutex> lg_color(this->mtx_img_color_);
    std::lock_guard<std::mutex> lg_depth(this->mtx_img_depth_);
    std::lock_guard<std::mutex> lg_cloud(this->mtx_cloud_);
    std::lock_guard<std::mutex> lg_pose(this->mtx_pose_);

    this->cv_img_cache_ = cv_bridge::toCvCopy(*(this->img_color_cache_));
    this->color_frame_ = cv_img_cache_->image;

    // TODO disable depth image processing
    // this->cv_img_cache_ = cv_bridge::toCvCopy(*(this->img_depth_cache_));
    // this->depth_frame_ = cv_img_cache_->image;

    this->cloudToLocalStorage();

    this->current_position_ = Eigen::Vector3f(
        this->pose_cache_->pose.position.x,
        this->pose_cache_->pose.position.y,
        this->pose_cache_->pose.position.z
    );
    this->current_orientation_ = Eigen::Vector4f(
        this->pose_cache_->pose.orientation.x,
        this->pose_cache_->pose.orientation.y,
        this->pose_cache_->pose.orientation.z,
        this->pose_cache_->pose.orientation.w
    );

    this->convertCloud();
    this->cloudFromLocalStorage();

    this->img_color_ready_ = false;
    this->img_depth_ready_ = false;
    this->cloud_ready_ = false;
    this->pose_ready_ = false;
}


void VisionModuleNode::detectTarget() {

    bool input_available = true;
    bool output_available = false;

    cv::Size img_size = this->color_frame_.size();
    if (img_size.empty()) {
        RCUTILS_LOG_WARN("[WARN] Could not retrive CV image size");
        input_available = false;
    } else if (
        (unsigned)img_size.width != this->tracker_config_.frame_width
        || (unsigned)img_size.height != this->tracker_config_.frame_height
    ) {
        RCUTILS_LOG_WARN(
            "[WARN] CV image does not meet expected frame size: (%d, %d) =/= (%d, %d)",
            img_size.width,
            img_size.height,
            this->tracker_config_.frame_width,
            this->tracker_config_.frame_height
        );
        input_available = false;
    }

    Eigen::Vector3f target_position(NAN, NAN, NAN);
    if (input_available) {
        target_position = this->tracker_->run(
            this->color_frame_,
            this->cloud_flattened_
        );
        if (target_position.x() == NAN || target_position.y() == NAN || target_position.z() == NAN)
            output_available = false;
        else
            output_available = true;
    }

    if (output_available) {
        this->msg_target_->vector.x = target_position.x();
        this->msg_target_->vector.y = target_position.y();
        this->msg_target_->vector.z = target_position.z();
        this->target_found_ = true;
    } else {
        this->msg_target_->vector.x = NAN;
        this->msg_target_->vector.y = NAN;
        this->msg_target_->vector.z = NAN;
        this->target_found_ = false;
    }
}


void VisionModuleNode::follow_target() {
    if (!this->node_config_.follow_mode) return;

    Eigen::Vector3f new_goal = this->tracker_->follow_target(this->current_position_);
    this->msg_goal_->vector.x = new_goal.x();
    this->msg_goal_->vector.y = new_goal.y();
    this->msg_goal_->vector.z = new_goal.z();
}


void VisionModuleNode::cloudToLocalStorage() {
    this->cloud_flattened_ = std::vector<float>(this->cloud_size_ * 3, 0.0f);

    if (this->cloud_cache_->data.size() < this->cloud_size_ * 3)
        RCUTILS_LOG_WARN(
            "[WARN] Incoming cloud size does not match expectation: %ld =/= %d",
            this->cloud_cache_->data.size(),
            this->cloud_size_ * 3
        );

    sensor_msgs::PointCloud2Iterator<float> iter_x(*(this->cloud_cache_), "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*(this->cloud_cache_), "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*(this->cloud_cache_), "z");
    for (unsigned i = 0, off = 0; i < this->cloud_size_; i++, off = i * 3) {
        this->cloud_flattened_[off]   = *(iter_x+i);
        this->cloud_flattened_[off+1] = *(iter_y+i);
        this->cloud_flattened_[off+2] = *(iter_z+i);
    }
}


void VisionModuleNode::cloudFromLocalStorage() {
    sensor_msgs::PointCloud2Iterator<float> iter_x(*(this->msg_cloud_), "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*(this->msg_cloud_), "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*(this->msg_cloud_), "z");
    for (unsigned i = 0, off = 0; i < this->cloud_size_; i++, off = i * 3) {
        *(iter_x+i) = this->cloud_flattened_[off];
        *(iter_y+i) = this->cloud_flattened_[off+1];
        *(iter_z+i) = this->cloud_flattened_[off+2];
    }
}


void VisionModuleNode::convertCloud() {
    Eigen::Quaternionf current_orientation_quat = Eigen::Quaternionf(
        this->current_orientation_.x(),
        this->current_orientation_.y(),
        this->current_orientation_.z(),
        this->current_orientation_.w()
    );
    Eigen::Matrix3f rotation_matrix = current_orientation_quat.normalized().toRotationMatrix();
    Eigen::Vector3f translation_vector = this->current_position_;

    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
    transformation_matrix.block<3, 3>(0, 0) = rotation_matrix;
    transformation_matrix.block<3, 1>(0, 3) = translation_vector;

    for (unsigned i = 0, off = 0; i < this->cloud_size_; i++, off = i * 3) {
        Eigen::Vector4f point_camera = Eigen::Vector4f(
            this->cloud_flattened_[off],
            this->cloud_flattened_[off+1],
            this->cloud_flattened_[off+2],
            1.0f
        );
        Eigen::Vector4f point_odom = transformation_matrix * point_camera;
        this->cloud_flattened_[off]   = point_odom.x();
        this->cloud_flattened_[off+1] = point_odom.z();
        this->cloud_flattened_[off+2] = point_odom.x();
    }
}


void VisionModuleNode::publish() {

    this->msg_target_->header.stamp = this->get_clock()->now();
    this->msg_target_->header.frame_id = "odom";
    this->pub_vision_target_->publish(*(this->msg_target_));

    this->msg_cloud_->header.stamp = this->get_clock()->now();
    this->msg_cloud_->header.frame_id = "odom";
    this->pub_vision_cloud_->publish(*(this->msg_cloud_));

    if (this->node_config_.follow_mode) {
        this->msg_goal_->header.stamp = this->get_clock()->now();
        this->msg_goal_->header.frame_id = "odom";
        this->pub_vision_goal_->publish(*(this->msg_goal_));
    }
}


void VisionModuleNode::declareRosParameters() {

    this->declare_parameter("ros_node.subs.camera_color", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.subs.camera_depth", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.subs.camera_cloud", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.subs.robot_pose", rclcpp::PARAMETER_STRING);

    this->declare_parameter("ros_node.pubs.vision_target", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.pubs.vision_cloud", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.pubs.vision_goal", rclcpp::PARAMETER_STRING);

    this->declare_parameter("ros_node.thread_freq", rclcpp::PARAMETER_DOUBLE);

    this->declare_parameter("follow.enable", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("follow.distance", rclcpp::PARAMETER_DOUBLE);

    this->declare_parameter("tracker.frame_width", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("tracker.frame_height", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("tracker.enable_cuda", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("tracker.yolo_input_width", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("tracker.yolo_input_height", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("tracker.yolo_model_path", rclcpp::PARAMETER_STRING);
    this->declare_parameter("tracker.yolo_labels_path", rclcpp::PARAMETER_STRING);
    this->declare_parameter("tracker.yolo_search_classes", rclcpp::PARAMETER_INTEGER_ARRAY);
    this->declare_parameter("tracker.yolo_min_confidence", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("tracker.mean_circle_radius", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("tracker.max_past_positions", rclcpp::PARAMETER_INTEGER);
}


void VisionModuleNode::initializeRosNodeConfig() {

    this->node_config_.sub_camera_color = this->get_parameter("ros_node.subs.camera_color").as_string();
    this->node_config_.sub_camera_depth = this->get_parameter("ros_node.subs.camera_depth").as_string();
    this->node_config_.sub_camera_cloud = this->get_parameter("ros_node.subs.camera_cloud").as_string();

    this->node_config_.pub_vision_target = this->get_parameter("ros_node.pubs.vision_target").as_string();
    this->node_config_.pub_vision_cloud = this->get_parameter("ros_node.pubs.vision_cloud").as_string();
    this->node_config_.pub_vision_goal = this->get_parameter("ros_node.pubs.vision_goal").as_string();

    this->node_config_.thread_freq = (float)(this->get_parameter("ros_node.thread_freq").as_double());
    this->node_config_.follow_mode = this->get_parameter("follow.enable").as_bool();
    
}


void VisionModuleNode::initializeComponents() {

    this->tracker_config_.frame_width  = (unsigned)(this->get_parameter("tracker.frame_width").as_int());
    this->tracker_config_.frame_height = (unsigned)(this->get_parameter("tracker.frame_height").as_int());
    this->tracker_config_.enable_cuda = this->get_parameter("tracker.enable_cuda").as_bool();
    this->tracker_config_.yolo_input_width  = (unsigned)(this->get_parameter("tracker.yolo_input_width").as_int());
    this->tracker_config_.yolo_input_height = (unsigned)(this->get_parameter("tracker.yolo_input_height").as_int());
    this->tracker_config_.yolo_model_path  = this->get_parameter("tracker.yolo_model_path").as_string();
    this->tracker_config_.yolo_labels_path = this->get_parameter("tracker.yolo_labels_path").as_string();
    this->tracker_config_.yolo_min_confidence = (float)(this->get_parameter("tracker.yolo_min_confidence").as_double());
    this->tracker_config_.mean_circle_radius = (unsigned)(this->get_parameter("tracker.mean_circle_radius").as_int());
    this->tracker_config_.max_past_positions = (unsigned)(this->get_parameter("tracker.max_past_positions").as_int());
    this->tracker_config_.execution_time = 1.0f / this->node_config_.thread_freq;

    std::vector<long> search_classes = this->get_parameter("tracker.yolo_search_classes").as_integer_array();
    this->tracker_config_.yolo_search_classes = {};
    for (const auto& cl : search_classes) {
        this->tracker_config_.yolo_search_classes.push_back((unsigned)cl);
    }

    this->tracker_config_.following_mode = this->get_parameter("follow.enable").as_bool();
    this->tracker_config_.following_distance = (float)this->get_parameter("follow.distance").as_double();

    this->tracker_ = std::make_unique<NAVIGATION_CORE::Tracker>(this->tracker_config_);

    this->cloud_size_ = this->tracker_config_.frame_width * this->tracker_config_.frame_height;
    this->cloud_flattened_ = std::vector<float>(this->cloud_size_ * 3);
    this->current_position_ = Eigen::Vector3f(NAN, NAN, NAN);
    this->current_orientation_ = Eigen::Vector4f(NAN, NAN, NAN, NAN);
}


void VisionModuleNode::initializeSubscribers() {

    this->sub_camera_color_ = this->create_subscription<sensor_msgs::msg::Image>(
        this->node_config_.sub_camera_color, 1, std::bind(&VisionModuleNode::cameraColorCallback, this, _1));
    // this->sub_camera_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
    //     this->node_config_.sub_camera_depth, 1, std::bind(&VisionModuleNode::cameraDepthCallback, this, _1)); // TODO disable depth image processing
    this->sub_camera_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        this->node_config_.sub_camera_cloud, 1, std::bind(&VisionModuleNode::cameraCloudCallback, this, _1));
    this->sub_robot_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->node_config_.sub_robot_pose, 1, std::bind(&VisionModuleNode::robotPoseCallback, this, _1));

    this->img_color_cache_ = std::make_shared<sensor_msgs::msg::Image>();
    // this->img_depth_cache_ = std::make_shared<sensor_msgs::msg::Image>(); // TODO disable depth image processing
    this->cloud_cache_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    this->pose_cache_ = std::make_shared<geometry_msgs::msg::PoseStamped>();
}


void VisionModuleNode::initializePublishers() {

    this->pub_vision_target_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(this->node_config_.pub_vision_target, 1);
    this->pub_vision_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->node_config_.pub_vision_cloud, 1);

    this->msg_target_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
    this->msg_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // initialize point cloud msg
    this->msg_cloud_->width = this->tracker_config_.frame_width;
    this->msg_cloud_->height = this->tracker_config_.frame_height;
    this->msg_cloud_->point_step = 12; // 12 bytes (3x float32)
    this->msg_cloud_->row_step = this->tracker_config_.frame_width * 12;
    this->msg_cloud_->is_bigendian = false;
    this->msg_cloud_->is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(*(this->msg_cloud_));
    modifier.setPointCloud2Fields(
        3,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32
    );
    modifier.resize(this->msg_cloud_->width * this->msg_cloud_->height);

    unsigned max_size = this->msg_cloud_->width * this->msg_cloud_->height;
    sensor_msgs::PointCloud2Iterator<float> iter_x(*(this->msg_cloud_), "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*(this->msg_cloud_), "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*(this->msg_cloud_), "z");
    for (unsigned i = 0; i < max_size; i++) {
        *(iter_x+i) = INFINITY;
        *(iter_y+i) = INFINITY;
        *(iter_z+i) = INFINITY;
    }

    if (this->node_config_.follow_mode) {
        this->pub_vision_goal_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(this->node_config_.pub_vision_goal, 1);
        this->msg_goal_ = std::make_shared<geometry_msgs::msg::Vector3Stamped>();
    }
}


void VisionModuleNode::initializeExecutionThread() {
    this->execute_rate_ = std::make_unique<rclcpp::Rate>(this->node_config_.thread_freq);
    this->execute_worker_ = std::thread(&VisionModuleNode::executeThread, this);
}


} // namespace NAVIGATION_ROS
