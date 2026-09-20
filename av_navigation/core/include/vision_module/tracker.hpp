#pragma once

#include <string>
#include <cmath>
#include <memory>

#include "eigen3/Eigen/Dense"
#include "opencv2/opencv.hpp"

#include "yolos/yolos.hpp"


namespace NAVIGATION_CORE {


inline int sqr(int x) {return x*x;}


struct TrackerConfig {

    unsigned frame_width = 1280;
    unsigned frame_height = 720;

    bool enable_cuda = true;
    std::string yolo_model_path = "yolo26m.onnx";
    std::string yolo_labels_path = "coco.names";
    unsigned yolo_input_width = 640;
    unsigned yolo_input_height = 640;
    std::vector<unsigned> yolo_search_classes = {2, 7}; // COCO class, default: "car", "truck"
    float yolo_min_confidence = 0.5f; // probability in range (0,1)
    
    unsigned mean_circle_radius = 10; // [px]
    unsigned max_past_positions = 10;
    float execution_time = 0.1; // [s]

    bool following_mode = false;
    float following_distance = 0.3; // [m]
};

class Tracker {

public:

    Tracker() = delete;
    Tracker(TrackerConfig config);
    ~Tracker() = default;

    Eigen::Vector3f run(
        const cv::Mat& image,
        const std::vector<float>& cloud
    );

    Eigen::Vector3f follow_target(
        const Eigen::Vector3f position
    );

    friend class TrackerTestClass;

protected:

    void initialize_();

    std::vector<yolos::Detection> detect_target_on_image_(
        const cv::Mat& image
    ) const;

    Eigen::Vector3f find_current_target_(
        const std::vector<yolos::Detection> detections,
        const std::vector<float>& cloud
    ) const;

    void predict_future_target_();

    TrackerConfig config_ = {};

    std::unique_ptr<yolos::YOLO26Detector> detector_;

    cv::Mat input_ = {};
    Eigen::Vector3f current_target_ = {};
    Eigen::Vector3f predicted_target_ = {};
    std::vector<Eigen::Vector3f> target_array_ = {};
};


}  // namesapce NAVIGATION_CORE
