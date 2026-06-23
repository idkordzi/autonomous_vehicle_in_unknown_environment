cmake_minimum_required(VERSION 3.15)
project(av_navigation_core VERSION 1.0.0)

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

if(NOT DEFINED AV_NAVIGATION_CORE_LIB_ROOT_PATH)
  set(AV_NAVIGATION_CORE_LIB_ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
  message("Core algorithm root path set to ${AV_NAVIGATION_CORE_LIB_ROOT_PATH}")
endif()

# ################################################################
# Download external libraries
# ################################################################

# Download ONNX runtime
if(NOT DEFINED ONNXRUNTIME_LIB_ROOT_PATH)
  set(ONNXRUNTIME_LIB_ROOT_PATH "${CMAKE_CURRENT_BINARY_DIR}/_deps/lib_onnx")
  set(ONNXRUNTIME_DIR "${ONNXRUNTIME_LIB_ROOT_PATH}")
  if (NOT EXISTS ${ONNXRUNTIME_DIR})
    message("ONNX runtime root path set to ${ONNXRUNTIME_LIB_ROOT_PATH}")
    file(MAKE_DIRECTORY ${ONNXRUNTIME_LIB_ROOT_PATH})
    execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/onnx_download.sh 1.20.1 1 ${ONNXRUNTIME_LIB_ROOT_PATH})
  endif()
endif()

# Download Eigen library
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

# ################################################################
# Find external libraries
# ################################################################

find_package(OpenCV REQUIRED)

# ################################################################
# Build libraries
# ################################################################

# Add camera_wrapper
set(TargetCameraWrapper drone_camera_wrapper)
add_library(${TargetCameraWrapper}
  STATIC
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/src/vision_module/camera_wrapper.cpp
)
target_compile_features(${TargetCameraWrapper} PUBLIC cxx_std_17)
target_include_directories(${TargetCameraWrapper} 
  PUBLIC 
    ${OpenCV_INCLUDE_DIRS} 
    ${EIGEN_LIB_ROOT_PATH} 
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/include/vision_module 
)
target_link_libraries(${TargetCameraWrapper} 
  PUBLIC 
    ${OpenCV_LIBS} 
)

# Add yolo_wrapper
set(TargetYoloWrapper drone_yolo_wrapper)
add_library(${TargetYoloWrapper}
  STATIC
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/src/vision_module/yolo_wrapper.cpp
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/src/vision_module/yolo_detector.cpp
)
target_compile_features(${TargetYoloWrapper} PUBLIC cxx_std_17)
target_include_directories(${TargetYoloWrapper} 
  PUBLIC 
    ${OpenCV_INCLUDE_DIRS} 
    ${ONNXRUNTIME_DIR}/include 
    ${EIGEN_LIB_ROOT_PATH} 
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/include/vision_module 
)
target_link_libraries(${TargetYoloWrapper} 
  PUBLIC 
    ${OpenCV_LIBS} 
    ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so 
)

# Add vfh_planner CUDA kernels
set(TargetVfhPlannerKernels drone_vfh_planner_kernels)
add_library(${TargetVfhPlannerKernels}
  STATIC
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/src/planner_module/local_planner_kernels.cu
)
target_compile_features(${TargetVfhPlannerKernels} PUBLIC cxx_std_17 cuda_std_17)
target_include_directories(${TargetVfhPlannerKernels} 
  PUBLIC 
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/include/planner_module 
)

# Add vfh_planner
set(TargetVfhPlanner drone_vfh_planner)
add_library(${TargetVfhPlanner}
  STATIC
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/src/planner_module/local_planner.cpp 
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/src/planner_module/polar_histogram.cpp
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/src/planner_module/common.cpp
)
target_compile_features(${TargetVfhPlanner} PUBLIC cxx_std_17)
target_include_directories(${TargetVfhPlanner} 
  PUBLIC 
    ${OpenCV_INCLUDE_DIRS} 
    ${EIGEN_LIB_ROOT_PATH} 
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/include/planner_module 
)
target_link_libraries(${TargetVfhPlanner} 
  PUBLIC 
    ${OpenCV_LIBS} 
    ${TargetVfhPlannerKernels} 
)

# Add flight_controller
set(TargetFlightController drone_flight_controller)
add_library(${TargetFlightController}
  STATIC
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/src/control_module/flight_controller.cpp
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/src/control_module/pid_controller.cpp
)
target_compile_features(${TargetFlightController} PUBLIC cxx_std_17)
target_include_directories(${TargetFlightController} 
  PUBLIC 
    ${EIGEN_LIB_ROOT_PATH} 
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/include/control_module 
)

# Add state_estimator
set(TargetStateEstimator drone_state_estimator)
add_library(${TargetStateEstimator}
  STATIC
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/src/control_module/state_estimator.cpp
)
target_compile_features(${TargetStateEstimator} PUBLIC cxx_std_17)
target_include_directories(${TargetStateEstimator} 
  PUBLIC 
    ${EIGEN_LIB_ROOT_PATH} 
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/include/control_module 
)

# Add UDP socket
set(TargetSocketUDP drone_socket_udp)
add_library(${TargetSocketUDP}
  STATIC
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/src/control_module/socket_udp.cpp
)
target_compile_features(${TargetSocketUDP} PUBLIC cxx_std_17)
target_include_directories(${TargetSocketUDP} 
  PUBLIC 
    ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/include/control_module 
)

# ################################################################
# Build executables (test classes)
# ################################################################

set(TargetDroneVisionTest drone_vision_test)
add_executable(${TargetDroneVisionTest} 
  ${OpenCV_INCLUDE_DIRS} 
  ${ONNXRUNTIME_DIR}/include 
  ${EIGEN_LIB_ROOT_PATH} 
  ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/test/vision_module/drone_vision_test_class.cpp 
)
target_compile_features(${TargetDroneVisionTest} PUBLIC cxx_std_17)
target_link_libraries(${TargetDroneVisionTest} 
  PUBLIC 
    ${OpenCV_LIBS} 
    ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so 
    ${TargetYoloWrapper} 
)

set(TargetDronePlannerTest drone_planner_test)
add_executable(${TargetDronePlannerTest} 
  ${OpenCV_INCLUDE_DIRS} 
  ${EIGEN_LIB_ROOT_PATH} 
  ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/test/planner_module/drone_planner_test_class.cpp 
)
target_compile_features(${TargetDronePlannerTest} PUBLIC cxx_std_17)
target_link_libraries(${TargetDronePlannerTest} 
  PUBLIC 
    ${OpenCV_LIBS} 
    ${TargetVfhPlanner} 
)

set(TargetDroneControllerTest drone_controller_test)
add_executable(${TargetDroneControllerTest} 
  ${OpenCV_INCLUDE_DIRS} 
  ${EIGEN_LIB_ROOT_PATH} 
  ${VEHICLE_NAVIGATION_CORE_LIB_ROOT_PATH}/test/control_module/drone_control_test_class.cpp 
)
target_compile_features(${TargetDroneControllerTest} PUBLIC cxx_std_17)
target_link_libraries(${TargetDroneControllerTest} 
  PUBLIC 
    ${OpenCV_LIBS} 
    ${TargetFlightController} 
    ${TargetStateEstimator} 
)

# ################################################################
# Install components
# ################################################################

if(BUILD_FOR_ROS)
  install(TARGETS 
    ${TargetCameraWrapper} 
    ${TargetYoloWrapper} 
    ${TargetVfhPlanner} 
    ${TargetFlightController} 
    ${TargetStateEstimator} 
    ${TargetSocketUDP} 
    DESTINATION lib/${PROJECT_NAME}
  )

  install(
    DIRECTORY
      yolo/models/
    DESTINATION share/${PROJECT_NAME}/yolo/models
  )

  install(
    DIRECTORY
      yolo/labels/
    DESTINATION share/${PROJECT_NAME}/yolo/labels
  )
endif()