#include "yolo_wrapper.hpp"


namespace NAVIGATION_CORE {

YOLOWrapper::YOLOWrapper() {
    this->initialize();
}

YOLOWrapper::YOLOWrapper(YOLOWrapperConfig config) : config_(config) {
    this->initialize();
}

void YOLOWrapper::initialize() {
    this->detector_ = std::make_unique<YOLO_DETECTOR::YOLO12Detector>(this->config_.model_path, this->config_.labels_path, this->config_.en_cuda);
    this->input_    = cv::Mat(this->config_.yolo_in_height, this->config_.yolo_in_width, CV_8UC3, cv::Scalar(0, 0, 0));

    this->target_loc_.fill(-1);

    this->target_last_loc_.x() = this->config_.yolo_in_width / 2;
    this->target_last_loc_.y() = this->config_.yolo_in_height / 2;
}

void YOLOWrapper::setInput(const cv::Mat& image) {
    image.copyTo(this->input_);
}

Eigen::Vector2i YOLOWrapper::getTarget() const {
    return this->target_loc_;
}

void YOLOWrapper::run() {
  std::vector<YOLO_DETECTOR::Detection> results = this->detector_->detect(this->input_);

    // Get detection with highest confidence 
    int bresult = -1;
    int bdist = sqr(this->config_.yolo_in_width) + sqr(this->config_.yolo_in_height);
    float bconf = 0.0f;
    for (unsigned i = 0; i < results.size(); i++) {
        if (results[i].classId == this->config_.yolo_class && results[i].conf > config_.yolo_min_conf) {
            int tx = std::floor(results[i].box.x + results[i].box.width / 2);
            int ty = std::floor(results[i].box.y + results[i].box.height / 2);
            int tdist = sqr(this->target_last_loc_.x() - tx) + sqr(this->target_last_loc_.y() - ty);
            if (tdist < bdist) {
                bresult = i;
                bdist = tdist;
                bconf = results[i].conf;
            } else if (tdist == bdist && results[i].conf > bconf) {
                bresult = i;
                bdist = tdist;
                bconf = results[i].conf;
            }
        }
    }
    if (bresult != -1) {
        this->target_last_loc_.x() = this->target_loc_.x();
        this->target_last_loc_.y() = this->target_loc_.y();

        this->target_loc_.x() = std::floor(results[bresult].box.x + results[bresult].box.width / 2);
        this->target_loc_.y() = std::floor(results[bresult].box.y + results[bresult].box.height / 2);
    }
    else this->target_loc_.fill(-1);
}

}  // namespace NAVIGATION_CORE
