#pragma once

#include <string>
#include <cmath>
#include <memory>

#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

#include "yolo_detector.hpp"


namespace NAVIGATION_CORE {

inline int sqr(int x) {return x*x;}

struct YOLOWrapperConfig {

    // yolo params
    bool en_cuda = true;
    unsigned yolo_in_width = 640;
    unsigned yolo_in_height = 360;

    int   yolo_class = 2;
    float yolo_min_conf = 0.6f; // (0,1)

    std::string model_path  = "src/yolo/models/yolo11n.onnx";
    std::string labels_path = "src/yolo/labels/coco.names";
};

class YOLOWrapper {

public:

    YOLOWrapper();
    YOLOWrapper(YOLOWrapperConfig config);
    ~YOLOWrapper() = default;

    void setInput(const cv::Mat& image);

    void run();

    Eigen::Vector2i getTarget() const;

protected:

    void initialize();

    YOLOWrapperConfig config_ = {};

    std::unique_ptr<YOLO_DETECTOR::YOLO12Detector> detector_;

    cv::Mat input_;
    Eigen::Vector2i target_loc_ = {};
    Eigen::Vector2i target_last_loc_ = {};
};

}  // namesapce NAVIGATION_CORE
