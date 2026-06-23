cmake_minimum_required(VERSION 3.15)
project(av_navigation_ros VERSION 1.0.0)

if (NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 86)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
enable_language(CUDA)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ################################################################
# Add external libraries
# ################################################################

# Download Eigen
if(NOT DEFINED EIGEN_LIB_ROOT_PATH)
  include(FetchContent)
  set(EIGEN_LIB_ROOT_PATH "${CMAKE_CURRENT_BINARY_DIR}/_deps/lib_eigen_src")
  message("Eigen library root path set to ${EIGEN_LIB_ROOT_PATH}")
  if (NOT EXISTS ${EIGEN_LIB_ROOT_PATH})
    FetchContent_Declare(
      lib-eigen
      GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
      GIT_TAG 3.4.0
      SOURCE_DIR "${EIGEN_LIB_ROOT_PATH}")
    FetchContent_Populate(lib-eigen)
  endif()
endif()

# # Download OpenCV library
# if(NOT DEFINED OPENCV_LIB_ROOT_PATH)
#   include(FetchContent)
#   set(OPENCV_LIB_ROOT_PATH "${CMAKE_CURRENT_BINARY_DIR}/_deps/lib_opencv_src")
#   message("OpenCV library root path set to ${OPENCV_LIB_ROOT_PATH}")
#   FetchContent_Declare(
#     lib-opencv
#     GIT_REPOSITORY https://github.com/opencv/opencv.git
#     GIT_TAG 4.11.0
#     SOURCE_DIR "${OPENCV_LIB_ROOT_PATH}")
#   FetchContent_MakeAvailable(lib-opencv)
# endif()

find_package(OpenCV REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drone_navigation_msgs REQUIRED)

find_package(cv_bridge REQUIRED)

# ################################################################
# Add external projects
# ################################################################

# Add core algorithm
set(BUILD_FOR_ROS true)
set(AV_NAVIGATION_CORE_LIB_ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/core")
add_subdirectory(core)

# ################################################################
# Build nodes
# ################################################################

# Add av_vision_node
set(TargetAvVisionNode av_vision_node)
add_executable(${TargetAvVisionNode} 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/vision_module_ros/vision_node_exec.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/vision_module_ros/vision_node.cpp 
)
target_compile_features(${TargetAvVisionNode} PUBLIC cxx_std_17)
ament_target_dependencies(${TargetAvVisionNode} 
  PUBLIC 
    rclcpp 
    rclcpp_components 
    geometry_msgs 
    sensor_msgs 
    cv_bridge 
)
target_include_directories(${TargetAvVisionNode} 
  PUBLIC 
    ${OpenCV_INCLUDE_DIRS} 
    ${cv_bridge_INCLUDE_DIRS} 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vision_module_ros 
)
target_link_libraries(${TargetAvVisionNode} 
  PUBLIC 
    ${OpenCV_LIBS} 
    ${cv_bridge_LIBS} 
    drone_camera_wrapper 
    drone_yolo_wrapper 
)

# Add av_planner_node
set(TargetAvPlannerNode av_planner_node)
add_executable(${TargetAvPlannerNode} 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/planner_module_ros/planner_node_exec.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/planner_module_ros/planner_node.cpp 
)
target_compile_features(${TargetAvPlannerNode} PUBLIC cxx_std_17)
ament_target_dependencies(${TargetAvPlannerNode} 
  PUBLIC 
    rclcpp 
    rclcpp_components 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    cv_bridge 
)
target_include_directories(${TargetAvPlannerNode} 
  PUBLIC 
    ${OpenCV_INCLUDE_DIRS} 
    ${EIGEN_LIB_ROOT_PATH} 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/planner_module_ros 
)
target_link_libraries(${TargetAvPlannerNode} 
  PUBLIC 
    ${OpenCV_LIBS} 
    ${cv_bridge_LIBS} 
    drone_vfh_planner 
)

# Add av_control_node
set(TargetAvControlNode av_control_node)
add_executable(${TargetAvControlNode} 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/control_module_ros/control_node_exec.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/control_module_ros/control_node.cpp 
)
target_compile_features(${TargetAvControlNode} PUBLIC cxx_std_17)
ament_target_dependencies(${TargetAvControlNode} 
  PUBLIC 
    rclcpp 
    rclcpp_components 
    tf2 
    tf2_geometry_msgs 
    geometry_msgs 
    sensor_msgs 
    drone_navigation_msgs 
)
target_include_directories(${TargetAvControlNode} 
  PUBLIC 
    ${EIGEN_LIB_ROOT_PATH} 
    ${CMAKE_CURRENT_SOURCE_DIR}/include/control_module_ros 
)
target_link_libraries(${TargetAvControlNode} 
  PUBLIC 
    drone_flight_controller 
    drone_state_estimator 
    drone_socket_udp 
)

# ################################################################
# Install files
# ################################################################

install(TARGETS 
  ${TargetAvVisionNode} 
  ${TargetAvPlannerNode} 
  ${TargetAvControlNode} 
  DESTINATION lib/${PROJECT_NAME} 
)

install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()