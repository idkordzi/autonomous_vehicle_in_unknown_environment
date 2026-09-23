#include "tracker.hpp"


namespace NAVIGATION_CORE {


Tracker::Tracker(TrackerConfig config) : config_(config) {
    this->initialize_();
}


void Tracker::initialize_() {
    if (this->config_.mode == 1) {
            this->detector_ = std::make_unique<yolos::YOLO26Detector>(
            this->config_.yolo_model_path,
            this->config_.yolo_labels_path,
            this->config_.enable_cuda
        );
    }
}


std::vector<yolos::Detection> Tracker::detect_target_on_image_(
    const cv::Mat& image
) const {

    std::vector<yolos::Detection> results_filtered = {};
    if (this->config_.mode == 0) {

        cv::Mat image_ycc;
        cv::cvtColor(image, image_ycc, cv::COLOR_BGR2YCrCb);

        unsigned
            cb_min = this->config_.color_cb_mean - this->config_.color_cb_dev,
            cb_max = this->config_.color_cb_mean + this->config_.color_cb_dev,
            cr_min = this->config_.color_cr_mean - this->config_.color_cr_dev,
            cr_max = this->config_.color_cr_mean + this->config_.color_cr_dev;
        int
            b_left = INT_MAX,
            b_right = INT_MIN,
            b_up = INT_MAX,
            b_down = INT_MIN;
        bool detected = false;

        for (int ri = 0; ri < (int)this->config_.frame_height; ri++) {
            for (int wi = 0; wi < (int)this->config_.frame_width; wi++) {
                cv::Vec<uint8_t, 3> px = image_ycc.at< cv::Vec<uint8_t, 3> >(ri, wi);
                if (px[1] > cb_min && px[1] < cb_max && px[2] > cr_min && px[2] < cr_max) {
                    if (wi < b_left) b_left = wi;
                    if (wi > b_right) b_right = wi;
                    if (ri < b_up) b_up = ri;
                    if (ri > b_down) b_down = ri;
                    detected = true;
                }
            }
        }
        if (detected) {
            results_filtered.push_back(
                yolos::Detection(
                    yolos::BoundingBox(
                        b_left,
                        b_up,
                        b_right-b_left,
                        b_down-b_up
                    ),
                    1.0f,
                    (int)this->config_.yolo_search_classes[0]
                )
            );
        }
    }

    else if (this->config_.mode == 1) {

        cv::Mat image_rgb;
        cv::cvtColor(image, image_rgb, cv::COLOR_BGR2RGB);

        std::vector<yolos::Detection> results =
            this->detector_->detect(image_rgb, this->config_.yolo_min_confidence);
        for (unsigned i = 0; i < results.size(); i++) {
            for (unsigned c : this->config_.yolo_search_classes) {
                if (results[i].classId == (int)c)
                    results_filtered.push_back(results[i]);
            }
        }
    }

    return results_filtered;
}


Eigen::Vector3f Tracker::find_current_target_(
    const std::vector<yolos::Detection> detections,
    const std::vector<float>& cloud
) const {

    std::vector<Eigen::Vector3f> possible_targets = {};
    unsigned r =  this->config_.mean_circle_radius;
    unsigned w = this->config_.frame_width;
    unsigned h = this->config_.frame_height;
    for (const yolos::Detection& detection : detections) {
        unsigned cx = detection.box.x + detection.box.width / 2;
        unsigned cy = detection.box.y + detection.box.height / 2;
        unsigned min_x = cx - r > 0 ? cx - r : 0;
        unsigned max_x = cx + r < w ? cx + r : w;
        unsigned min_y = cy - r > 0 ? cy - r : 0;
        unsigned max_y = cy + r < h ? cy + r : h;
        float px = 0.0f, py = 0.0f, pz = 0.0f, cn = 0.0f;
        for (unsigned i = min_y; i < max_y; i ++) {
            for (unsigned j = min_x; j < max_x; j++) {
                unsigned off = (i * w + j) * 3;
                px += cloud[off];
                py += cloud[off+1];
                px += cloud[off+2];
                cn += 1.0;
            }
        }
        px /= cn;
        py /= cn;
        pz /= cn;
        possible_targets.push_back(Eigen::Vector3f(px, py, pz));
    }

    if (possible_targets.size() > 0){
        unsigned best_target = detections.size();
        float distance = INFINITY;
        for (unsigned i = 0; i < possible_targets.size(); i++) {
            float target_distance = (this->predicted_target_ - possible_targets[i]).norm();
            if (target_distance < distance) {
                distance = target_distance;
                best_target = i;
            }
        }
        if (best_target < possible_targets.size())
            return possible_targets[best_target];
    }
    return Eigen::Vector3f(NAN, NAN, NAN);
}


void Tracker::predict_future_target_() {

    if ((unsigned)this->target_array_.size() >= this->config_.max_past_positions)
        this->target_array_.pop_back();
    this->target_array_.insert(this->target_array_.cbegin(), this->current_target_);

    std::vector<Eigen::Vector3f> velocity = {};
    Eigen::Vector3f mean_velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    if (this->target_array_.size() > 1)
        for (unsigned i = 1; i < this->target_array_.size(); i++) {
            Eigen::Vector3f partial = Eigen::Vector3f(
                this->target_array_[i].x() - this->target_array_[i-1].x(),
                this->target_array_[i].y() - this->target_array_[i-1].y(),
                this->target_array_[i].z() - this->target_array_[i-1].z()
            );
            velocity.push_back(partial);
            mean_velocity += partial;
        }
    if (velocity.size() > 0)
        mean_velocity /= (float)velocity.size();

    std::vector<Eigen::Vector3f> acceleration = {};
    Eigen::Vector3f mean_acceleration = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    if (velocity.size() > 1)
        for (unsigned i = 1; i < velocity.size(); i++) {
            Eigen::Vector3f partial = Eigen::Vector3f(
                velocity[i].x() - velocity[i-1].x(),
                velocity[i].y() - velocity[i-1].y(),
                velocity[i].z() - velocity[i-1].z()
            );
            acceleration.push_back(partial);
            mean_acceleration += partial;
        }
    if (acceleration.size() > 0)
        mean_acceleration /= (float)acceleration.size();

    float t = this->config_.execution_time;
    this->predicted_target_ = this->current_target_ + (mean_velocity * t) + (mean_acceleration * t * t * 0.5);
}


Eigen::Vector3f Tracker::run(
    const cv::Mat& image,
    const std::vector<float>& cloud
) {

    std::vector<yolos::Detection> results_filtered = this->detect_target_on_image_(image);
    Eigen::Vector3f found_target = this->find_current_target_(results_filtered, cloud);

    if (std::isnan(found_target.x()) || std::isnan(found_target.x()) || std::isnan(found_target.x()))
        this->current_target_ = this->predicted_target_;
    else
        this->current_target_ = found_target;
    this->predict_future_target_();

    return this->current_target_;
}


Eigen::Vector3f Tracker::follow_target(
    const Eigen::Vector3f position
) {
    Eigen::Vector3f point = this->current_target_;
    float radi = (point - position).norm();
    if (radi < this->config_.following_distance) {
        return position;
    }
    
    float den = (point.topRows<2>() - position.topRows<2>()).norm();
    float elev = std::atan2(point.z() - position.z(), den);
    float azim = std::atan2(point.y() - position.y(), point.x() - position.x());
    float new_radi = radi - this->config_.following_distance;
    Eigen::Vector3f new_goal = Eigen::Vector3f(
        new_radi * std::cos(elev) * std::cos(azim),
        new_radi * std::cos(elev) * std::sin(azim),
        new_radi * std::sin(elev)
    );
    return new_goal;
}


}  // namespace NAVIGATION_CORE
