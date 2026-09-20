#include "mock_publisher_node.hpp"


using std::placeholders::_1;


namespace NAVIGATION_ROS {


MockPublisherNode::MockPublisherNode(rclcpp::NodeOptions options)
: Node("mock_publisher_node", options)
{
    this->declareRosParameters();
    this->initializeRosNodeConfig();
    this->initializePublishers();
    this->initializeExecutionThread();
}


MockPublisherNode::~MockPublisherNode() {
    this->worker_camera_.join();
    this->worker_imu_.join();
    this->worker_odometry_.join();
}


void MockPublisherNode::threadCameraPublisher() {
    while (rclcpp::ok()) {
        rclcpp::Time stamp = this->get_clock()->now();

        // image color
        this->msg_image_color_->header.stamp = stamp;
        this->msg_image_color_->header.frame_id = "camera";

        // image depth
        this->msg_image_depth_->header.stamp = stamp;
        this->msg_image_depth_->header.frame_id = "camera";

        // point cloud
        this->msg_point_cloud_->header.stamp = stamp;
        this->msg_point_cloud_->header.frame_id = "camera";

        // publish
        this->pub_camera_image_color_->publish(*(this->msg_image_color_));
        this->pub_camera_image_depth_->publish(*(this->msg_image_depth_));
        this->pub_camera_point_cloud_->publish(*(this->msg_point_cloud_));

        if (this->rate_camera_)
            this->rate_camera_->sleep();
    }
}


void MockPublisherNode::threadImuPublisher() {
    while (rclcpp::ok()) {
        rclcpp::Time stamp = this->get_clock()->now();

        this->msg_imu_data_->header.stamp = stamp;
        this->msg_imu_data_->header.frame_id = "odom";

        this->pub_imu_data_->publish(*( this->msg_imu_data_));

        if (this->rate_imu_)
            this->rate_imu_->sleep();
    }
}


void MockPublisherNode::threadOdometryPublisher() {
    while (rclcpp::ok()) {
        rclcpp::Time stamp = this->get_clock()->now();

        this->msg_odoemetry_wheels_->header.stamp = stamp;
        this->msg_odoemetry_wheels_->header.frame_id = "odom";

        this->pub_odometry_wheels_->publish(*( this->msg_odoemetry_wheels_));

        if (this->rate_odometry_)
            this->rate_odometry_->sleep();
    }
}


void MockPublisherNode::declareRosParameters() {

    this->declare_parameter("ros_node.sensors.camera.pubs.color", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.sensors.camera.pubs.depth", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.sensors.camera.pubs.cloud", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.sensors.camera.freq", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("ros_node.sensors.camera.image.width", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("ros_node.sensors.camera.image.height", rclcpp::PARAMETER_INTEGER);

    this->declare_parameter("ros_node.sensors.imu.pubs.data", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.sensors.imu.freq", rclcpp::PARAMETER_DOUBLE);

    this->declare_parameter("ros_node.sensors.odometry.pubs.wheels", rclcpp::PARAMETER_STRING);
    this->declare_parameter("ros_node.sensors.odometry.freq", rclcpp::PARAMETER_DOUBLE);
}


void MockPublisherNode::initializeRosNodeConfig() {

    this->node_config_.pub_camera_color = this->get_parameter("ros_node.sensors.camera.pubs.color").as_string();
    this->node_config_.pub_camera_depth = this->get_parameter("ros_node.sensors.camera.pubs.depth").as_string();
    this->node_config_.pub_camera_cloud = this->get_parameter("ros_node.sensors.camera.pubs.cloud").as_string();
    this->node_config_.camera_thread_freq = (float)(this->get_parameter("ros_node.sensors.camera.freq").as_double());
    this->node_config_.camera_image_width = this->get_parameter("ros_node.sensors.camera.image.width").as_int();
    this->node_config_.camera_image_height = this->get_parameter("ros_node.sensors.camera.image.height").as_int();

    this->node_config_.pub_imu_data = this->get_parameter("ros_node.sensors.imu.pubs.data").as_string();
    this->node_config_.imu_thread_freq = (float)(this->get_parameter("ros_node.sensors.imu.freq").as_double());

    this->node_config_.pub_odometry_wheels = this->get_parameter("ros_node.sensors.odometry.pubs.wheels").as_string();
    this->node_config_.odometry_thread_freq = (float)(this->get_parameter("ros_node.sensors.odometry.freq").as_double());
}


void MockPublisherNode::initializePublishers() {

    this->pub_camera_image_color_ = this->create_publisher<sensor_msgs::msg::Image>(this->node_config_.pub_camera_color, 1);
    this->pub_camera_image_depth_ = this->create_publisher<sensor_msgs::msg::Image>(this->node_config_.pub_camera_depth, 1);
    this->pub_camera_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(this->node_config_.pub_camera_cloud, 1);
    this->pub_imu_data_ = this->create_publisher<sensor_msgs::msg::Imu>(this->node_config_.pub_imu_data, 1);
    this->pub_odometry_wheels_ = this->create_publisher<nav_msgs::msg::Odometry>(this->node_config_.pub_odometry_wheels, 1);

    // init image color msg
    this->msg_image_color_ = std::make_shared<sensor_msgs::msg::Image>();
    this->msg_image_color_->width = this->node_config_.camera_image_width;
    this->msg_image_color_->height = this->node_config_.camera_image_height;
    this->msg_image_color_->encoding = sensor_msgs::image_encodings::RGB8;
    this->msg_image_color_->is_bigendian = 0;
    this->msg_image_color_->step = this->node_config_.camera_image_width * 3; // RGB

    std::vector<uint8_t> image_color_data =
        std::vector<uint8_t>(this->node_config_.camera_image_width * this->node_config_.camera_image_height * 3);
    this->msg_image_color_->data = image_color_data;

    // init image depth msg
    this->msg_image_depth_ = std::make_shared<sensor_msgs::msg::Image>();
    this->msg_image_depth_->width = this->node_config_.camera_image_width;
    this->msg_image_depth_->height = this->node_config_.camera_image_height;
    this->msg_image_depth_->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    this->msg_image_depth_->is_bigendian = 0;
    this->msg_image_depth_->step = this->node_config_.camera_image_width * 4; // RGB

    std::vector<uint8_t> image_depth_data =
        std::vector<uint8_t>(this->node_config_.camera_image_width * this->node_config_.camera_image_height * 4);
    this->msg_image_depth_->data = image_depth_data;

    // initialize pcd msg
    int bytes_per_pixel = 12; // 12 bytes (3x float32)
    unsigned max_size = this->node_config_.camera_image_width * this->node_config_.camera_image_height;

    this->msg_point_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    this->msg_point_cloud_->width = this->node_config_.camera_image_width;
    this->msg_point_cloud_->height = this->node_config_.camera_image_height;
    this->msg_point_cloud_->point_step = bytes_per_pixel;
    this->msg_point_cloud_->row_step = this->node_config_.camera_image_width * bytes_per_pixel;
    this->msg_point_cloud_->is_bigendian = false;
    this->msg_point_cloud_->is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(*(this->msg_point_cloud_));
    modifier.setPointCloud2Fields(
        3,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32
    );
    modifier.resize(this->msg_point_cloud_->width * this->msg_point_cloud_->height);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*(this->msg_point_cloud_), "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*(this->msg_point_cloud_), "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*(this->msg_point_cloud_), "z");
    for (unsigned i = 0; i < max_size; i++) {
        *(iter_x+i) = 0.0f;
        *(iter_y+i) = 0.0f;
        *(iter_z+i) = 0.0f;
    }


    // init imu msg
    this->msg_imu_data_ = std::make_shared<sensor_msgs::msg::Imu>();
    this->msg_imu_data_->orientation = geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(0.0);
    this->msg_imu_data_->orientation_covariance = {0.0};
    this->msg_imu_data_->angular_velocity = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0);
    this->msg_imu_data_->angular_velocity_covariance = {0.0};
    this->msg_imu_data_->linear_acceleration = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0);
    this->msg_imu_data_->linear_acceleration_covariance = {0.0};

    // init odom msg
    this->msg_odoemetry_wheels_ = std::make_shared<nav_msgs::msg::Odometry>();
    this->msg_odoemetry_wheels_->pose.pose.position = geometry_msgs::build<geometry_msgs::msg::Point>().x(0.0).y(0.0).z(0.0);
    this->msg_odoemetry_wheels_->pose.pose.orientation = geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(0.0);
    this->msg_odoemetry_wheels_->pose.covariance = {0.0};
    this->msg_odoemetry_wheels_->twist.twist.linear = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0);
    this->msg_odoemetry_wheels_->twist.twist.angular = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0);
    this->msg_odoemetry_wheels_->twist.covariance = {0.0};
}


void MockPublisherNode::initializeExecutionThread() {
    this->rate_camera_ = std::make_unique<rclcpp::Rate>(this->node_config_.camera_thread_freq);
    this->worker_camera_ = std::thread(&MockPublisherNode::threadCameraPublisher, this);

    this->rate_imu_ = std::make_unique<rclcpp::Rate>(this->node_config_.imu_thread_freq);
    this->worker_imu_ = std::thread(&MockPublisherNode::threadImuPublisher, this);

    this->rate_odometry_ = std::make_unique<rclcpp::Rate>(this->node_config_.odometry_thread_freq);
    this->worker_odometry_ = std::thread(&MockPublisherNode::threadOdometryPublisher, this);
}


} // namespace NAVIGATION_ROS
