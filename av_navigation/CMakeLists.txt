cmake_minimum_required(VERSION 3.15)
project(av_navigation_ros VERSION 1.0.0 LANGUAGES CXX CUDA)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
enable_language(CUDA)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 86)
endif()

# ################################################################
# Add external libraries
# ################################################################

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(cv_bridge REQUIRED)

# ################################################################
# Add external projects
# ################################################################

# Add core algorithm
set(BUILD_FOR_ROS true)
set(AV_NAVIGATION_CORE_ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/core")
add_subdirectory(core)

# ################################################################
# Build nodes
# ################################################################

# Add VISION node
set(TARGET_AV_VISION_NODE av_vision_node)
add_executable(${TARGET_AV_VISION_NODE}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vision_module_ros/vision_node.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vision_module_ros/vision_node_exec.cpp
)
target_compile_features(${TARGET_AV_VISION_NODE} PUBLIC cxx_std_17)
ament_target_dependencies(${TARGET_AV_VISION_NODE}
    PUBLIC
        rclcpp
        rclcpp_components
        geometry_msgs
        sensor_msgs
        cv_bridge
)
target_include_directories(${TARGET_AV_VISION_NODE}
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include/vision_module_ros
)
target_link_libraries(${TARGET_AV_VISION_NODE}
    PUBLIC
        ${OpenCV_LIBS}
        ${cv_bridge_LIBS}
        av_tracker
)

# Add PLANNER node
set(TARGET_AV_PLANNER_NODE av_planner_node)
add_executable(${TARGET_AV_PLANNER_NODE}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planner_module_ros/planner_node.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planner_module_ros/planner_node_exec.cpp
)
target_compile_features(${TARGET_AV_PLANNER_NODE} PUBLIC cxx_std_17)
ament_target_dependencies(${TARGET_AV_PLANNER_NODE}
    PUBLIC
        rclcpp
        rclcpp_components
        std_msgs
        geometry_msgs
        sensor_msgs
        cv_bridge
)
target_include_directories(${TARGET_AV_PLANNER_NODE}
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include/planner_module_ros
)
target_link_libraries(${TARGET_AV_PLANNER_NODE}
    PUBLIC
        ${OpenCV_LIBS}
        ${cv_bridge_LIBS}
        av_planner
)

# Add (POSE) ESTIMATION node
set(TARGET_ESTIMATION_NODE av_estimation_node)
add_executable(${TARGET_ESTIMATION_NODE}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/estimation_module_ros/estimation_node.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/estimation_module_ros/estimation_node_exec.cpp
)
target_compile_features(${TARGET_ESTIMATION_NODE} PUBLIC cxx_std_17)
ament_target_dependencies(${TARGET_ESTIMATION_NODE}
    PUBLIC
        rclcpp
        rclcpp_components
        geometry_msgs
        nav_msgs
)
target_include_directories(${TARGET_ESTIMATION_NODE}  
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/estimation_module_ros
)

# Add CONTROLLER node
set(TARGET_AV_CONTROL_NODE av_control_node)
add_executable(${TARGET_AV_CONTROL_NODE}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control_module_ros/control_node.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control_module_ros/control_node_exec.cpp
)
target_compile_features(${TARGET_AV_CONTROL_NODE} PUBLIC cxx_std_17)
ament_target_dependencies(${TARGET_AV_CONTROL_NODE}
    PUBLIC
        rclcpp
        rclcpp_components
        geometry_msgs
)
target_include_directories(${TARGET_AV_CONTROL_NODE}  
    PUBLIC
        ${Eigen_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include/control_module_ros
)
target_link_libraries(${TARGET_AV_CONTROL_NODE}
    PUBLIC
        av_controller
)

# Add MOCK PUBLISHER node
set(TARGET_MOCK_PUBLISHER_NODE mock_publisher_node)
add_executable(${TARGET_MOCK_PUBLISHER_NODE}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mock_publisher_ros/mock_publisher_node.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mock_publisher_ros/mock_publisher_node_exec.cpp
)
target_compile_features(${TARGET_MOCK_PUBLISHER_NODE} PUBLIC cxx_std_17)
ament_target_dependencies(${TARGET_MOCK_PUBLISHER_NODE}
    PUBLIC
        rclcpp
        rclcpp_components
        geometry_msgs
        sensor_msgs
        nav_msgs
)
target_include_directories(${TARGET_MOCK_PUBLISHER_NODE}  
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/mock_publisher_ros
)

# ################################################################
# Install files
# ################################################################

install(
    TARGETS
        ${TARGET_AV_VISION_NODE}
        ${TARGET_AV_PLANNER_NODE}
        ${TARGET_ESTIMATION_NODE}
        ${TARGET_AV_CONTROL_NODE}
        ${TARGET_MOCK_PUBLISHER_NODE}
    DESTINATION
        lib/${PROJECT_NAME}
)
install(
    DIRECTORY
        config/
    DESTINATION
        share/${PROJECT_NAME}/config
)
install(
    DIRECTORY
        launch/
    DESTINATION
        share/${PROJECT_NAME}/launch
)
install(
    DIRECTORY
        launch/
    DESTINATION
        share/${PROJECT_NAME}/launch
)
install(
    FILES
        yolo/coco.names
        yolo/yolo26m.onnx
    DESTINATION
        share/${PROJECT_NAME}/yolo
)

ament_package()
