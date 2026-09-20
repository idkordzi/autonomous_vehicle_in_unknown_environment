#pragma once

#include <string>
#include <array>
#include <vector>
#include <thread>
#include <mutex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"


namespace NAVIGATION_ROS {


struct MockPublisherNodeConfig {
    std::string pub_camera_color = "/robot/sensors/camera/color";
    std::string pub_camera_depth = "/robot/sensors/camera/depth";
    std::string pub_camera_cloud = "/robot/sensors/camera/cloud";
    float camera_thread_freq = 30.0f;
    unsigned camera_image_width = 1280;
    unsigned camera_image_height = 720;

    std::string pub_imu_data = "/robot/sensors/imu/data";
    float imu_thread_freq = 100.0f;

    std::string pub_odometry_wheels = "/robot/sensors/odometry/wheels";
    float odometry_thread_freq = 50.0f;
};


class MockPublisherNode : public rclcpp::Node {

public:
    MockPublisherNode(rclcpp::NodeOptions options);
    ~MockPublisherNode();

private:

    void declareRosParameters();
    void initializeRosNodeConfig();
    void initializePublishers();
    void initializeExecutionThread();

    void threadCameraPublisher();
    void threadImuPublisher();
    void threadOdometryPublisher();

    MockPublisherNodeConfig node_config_ = {};

    // publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_camera_image_color_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_camera_image_depth_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_camera_point_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_data_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_wheels_;

    // published msg cache
    sensor_msgs::msg::Image::SharedPtr msg_image_color_;
    sensor_msgs::msg::Image::SharedPtr msg_image_depth_;
    sensor_msgs::msg::PointCloud2::SharedPtr msg_point_cloud_;
    sensor_msgs::msg::Imu::SharedPtr msg_imu_data_;
    nav_msgs::msg::Odometry::SharedPtr msg_odoemetry_wheels_;

    // execution thread
    std::thread worker_camera_;
    std::thread worker_imu_;
    std::thread worker_odometry_;

    std::unique_ptr<rclcpp::Rate> rate_camera_;
    std::unique_ptr<rclcpp::Rate> rate_imu_;
    std::unique_ptr<rclcpp::Rate> rate_odometry_;
};


} // namespace NAVIGATION_ROS
