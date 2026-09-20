#include "mock_publisher_node.hpp"


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NAVIGATION_ROS::MockPublisherNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
