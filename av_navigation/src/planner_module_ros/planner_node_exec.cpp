#include "planner_node.hpp"


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NAVIGATION_ROS::PlannerModuleNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
