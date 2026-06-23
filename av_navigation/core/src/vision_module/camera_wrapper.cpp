#include "camera_wrapper.hpp"


namespace NAVIGATION_CORE {

CameraWrapper::CameraWrapper() {
    this->initialize();
}

CameraWrapper::CameraWrapper(CameraWrapperConfig config) : config_(config) {
    this->initialize();
}

void CameraWrapper::initialize() {
    // @TODO
}

cv::Mat CameraWrapper::getColorImage() {
    return cv::Mat(this->config_.im_height, this->config_.im_width, CV_8UC3);
}

cv::Mat CameraWrapper::getDepthImage() {
    return cv::Mat(this->config_.im_height, this->config_.im_width, CV_32FC1);
}

PointCloud2D CameraWrapper::getPointCloud() {
    return PointCloud2D(this->config_.im_height, this->config_.im_width);
}

} // namespace NAVIGATION_CORE
