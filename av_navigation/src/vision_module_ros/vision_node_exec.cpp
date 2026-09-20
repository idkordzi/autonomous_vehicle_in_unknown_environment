#include "vision_node.hpp"


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NAVIGATION_ROS::VisionModuleNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
