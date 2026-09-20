cmake_minimum_required(VERSION 3.15)
project(av_navigation_core VERSION 1.0.0 LANGUAGES CXX CUDA)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 86)
endif()

if(NOT DEFINED AV_NAVIGATION_CORE_ROOT_PATH)
    set(AV_NAVIGATION_CORE_ROOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
    message("Core algorithm root path set to ${AV_NAVIGATION_CORE_ROOT_PATH}")
endif()

# ################################################################
# Find external libraries
# ################################################################

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# ################################################################
# Download external libraries
# ################################################################

# Download ONNX runtime
if(NOT DEFINED ONNXRUNTIME_ROOT_PATH)
    set(ONNXRUNTIME_ROOT_PATH "${AV_NAVIGATION_CORE_ROOT_PATH}/onnxruntime")
    message("ONNX RUNTIME root path set to ${ONNXRUNTIME_ROOT_PATH}")
    if (NOT EXISTS ${ONNXRUNTIME_ROOT_PATH})
        message("ONNX RUNTIME root dir not found - attempting to downlaod")
        execute_process(COMMAND ${AV_NAVIGATION_CORE_ROOT_PATH}/onnx_download.sh 1.20.1 1 ${ONNXRUNTIME_ROOT_PATH})
    endif()
endif()

# ################################################################
# Build libraries
# ################################################################

# Add TRACKER
set(TARGET_AV_TRACKER av_tracker)
add_library(${TARGET_AV_TRACKER}
    STATIC
        ${AV_NAVIGATION_CORE_ROOT_PATH}/src/vision_module/tracker.cpp
)
target_compile_features(${TARGET_AV_TRACKER} PUBLIC cxx_std_17)
target_include_directories(${TARGET_AV_TRACKER}
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIR}
        ${ONNXRUNTIME_ROOT_PATH}/include
        ${AV_NAVIGATION_CORE_ROOT_PATH}/YOLOs-CPP/include
        ${AV_NAVIGATION_CORE_ROOT_PATH}/include/vision_module
)
target_link_libraries(${TARGET_AV_TRACKER}
    PUBLIC
        ${OpenCV_LIBS}
        ${ONNXRUNTIME_ROOT_PATH}/lib/libonnxruntime.so
)

# Add PLANNER CUDA kernels
set(TARGET_AV_PLANNER_CUDA av_planner_cuda)
add_library(${TARGET_AV_PLANNER_CUDA}
    STATIC
        ${AV_NAVIGATION_CORE_ROOT_PATH}/src/planner_module/local_planner_kernels.cu
)
target_compile_features(${TARGET_AV_PLANNER_CUDA} PUBLIC cxx_std_17 cuda_std_17)
target_compile_options(${TARGET_AV_PLANNER_CUDA} PUBLIC -Wno-pedantic)
target_include_directories(${TARGET_AV_PLANNER_CUDA}
    PUBLIC
        ${AV_NAVIGATION_CORE_ROOT_PATH}/include/planner_module
)

# Add PLANNER
set(TARGET_AV_PLANNER av_planner)
add_library(${TARGET_AV_PLANNER}
    STATIC
        ${AV_NAVIGATION_CORE_ROOT_PATH}/src/planner_module/common.cpp
        ${AV_NAVIGATION_CORE_ROOT_PATH}/src/planner_module/polar_histogram.cpp
        ${AV_NAVIGATION_CORE_ROOT_PATH}/src/planner_module/local_planner.cpp 
)
target_compile_features(${TARGET_AV_PLANNER} PUBLIC cxx_std_17)
target_include_directories(${TARGET_AV_PLANNER}
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIR}
        ${AV_NAVIGATION_CORE_ROOT_PATH}/include/planner_module
)
target_link_libraries(${TARGET_AV_PLANNER}
  PUBLIC
    ${OpenCV_LIBS}
    ${TARGET_AV_PLANNER_CUDA}
)

# Add CONTROLLER
set(TARGET_AV_CONTROLLER av_controller)
add_library(${TARGET_AV_CONTROLLER}
    STATIC
        ${AV_NAVIGATION_CORE_ROOT_PATH}/src/control_module/pid.cpp
        ${AV_NAVIGATION_CORE_ROOT_PATH}/src/control_module/controller.cpp
)
target_compile_features(${TARGET_AV_CONTROLLER} PUBLIC cxx_std_17)
target_include_directories(${TARGET_AV_CONTROLLER}
    PUBLIC
        ${Eigen_INCLUDE_DIR}
        ${AV_NAVIGATION_CORE_ROOT_PATH}/include/control_module
)

# ################################################################
# Build executables (tests)
# ################################################################

# Test VISION
set(TARGET_AV_VISION_TEST av_vision_test)
add_executable(${TARGET_AV_VISION_TEST}
    ${AV_NAVIGATION_CORE_ROOT_PATH}/test/vision_module/vision_module_test.cpp
)
target_compile_features(${TARGET_AV_VISION_TEST} PUBLIC cxx_std_17)
target_include_directories(${TARGET_AV_VISION_TEST}
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIR}
        ${ONNXRUNTIME_ROOT_PATH}/include
)
target_link_libraries(${TARGET_AV_VISION_TEST} 
    PUBLIC
        ${OpenCV_LIBS}
        ${ONNXRUNTIME_ROOT_PATH}/lib/libonnxruntime.so
        ${TARGET_AV_TRACKER}
)

# Test PLANNER
set(TARGET_AV_PLANNER_TEST av_planner_test)
add_executable(${TARGET_AV_PLANNER_TEST}
    ${AV_NAVIGATION_CORE_ROOT_PATH}/test/planner_module/planner_module_test.cpp
)
target_compile_features(${TARGET_AV_PLANNER_TEST} PUBLIC cxx_std_17)
target_include_directories(${TARGET_AV_PLANNER_TEST}
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIR}
)
target_link_libraries(${TARGET_AV_PLANNER_TEST}
    PUBLIC
        ${OpenCV_LIBS}
        ${TARGET_AV_PLANNER}
)

# Test CONTROLLER
set(TARGET_AV_CONTROL_TEST av_control_test)
add_executable(${TARGET_AV_CONTROL_TEST}
    ${AV_NAVIGATION_CORE_ROOT_PATH}/test/control_module/control_module_test.cpp
)
target_compile_features(${TARGET_AV_CONTROL_TEST} PUBLIC cxx_std_17)
target_include_directories(${TARGET_AV_CONTROL_TEST}
    PUBLIC
        ${Eigen_INCLUDE_DIR}
)
target_link_libraries(${TARGET_AV_CONTROL_TEST}
    PUBLIC
        ${TARGET_AV_CONTROLLER}
)

# ################################################################
# Install components
# ################################################################

if(BUILD_FOR_ROS)
    install(
        TARGETS 
            ${TARGET_AV_TRACKER} 
            ${TARGET_AV_PLANNER} 
            ${TARGET_AV_CONTROLLER}  
        DESTINATION
            lib/${PROJECT_NAME}
    )
endif()
