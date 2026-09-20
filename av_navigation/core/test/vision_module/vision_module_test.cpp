#include <iostream>
#include <set>
#include <vector>
#include <string>
#include <memory>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Dense"

#include "tracker.hpp"


namespace NAVIGATION_CORE {


inline bool eqm(float a, float b, float m) {
    return std::abs(a-b) <= m;
}


class TrackerTestClass {

public:

    TrackerTestClass() = default;
    ~TrackerTestClass() = default;

    void initTracker(TrackerConfig config) {
        std::cout << "[INFO] Initializing TRACKER class\n";
        this->tracker_ = std::make_unique<Tracker>(config);
    }

    void testReadImage(const std::string image_path, bool display_result = true) {
        std::cout << "[TEST] Read image from file\n";

        cv::Mat image = this->loadImage(image_path);
        if (image.empty()) return;

        std::cout << "[SUCCESS] Image size: (" << image.size[0] << "x" << image.size[1] << ")\n";
        if (display_result) {
            std::cout << "[INFO] Display image preview - press any key to continue\n";
            cv::imshow("Image preview", image);
            cv::waitKey(0);
            cv::destroyWindow("Image preview");
        }
    }

    void testReadLabels(const std::string labels_path) {
        std::cout << "[TEST] Read labels from file\n";

        std::vector<std::string> labels = this->loadLabels(labels_path);
        if (labels.size() == 0) return;

        std::cout << "[SUCCESS] Numbers of found labels: " << labels.size() << "\n";
    }

    void testInference(const std::string image_path, const std::string labels_path) {
        std::cout << "[TEST] Run inferance\n";

        cv::Mat image = this->loadImage(image_path);
        if (image.empty()) return;
        std::vector<std::string> labels = this->loadLabels(labels_path);
        if (labels.size() == 0) return;

        float min_confidence = this->tracker_->config_.yolo_min_confidence;
        std::vector<yolos::Detection> results = this->tracker_->detector_->detect(image, min_confidence);
        if (results.size() == 0) {
            std::cout << "[ERROR] Empty detection results!\n";
            return;
        }
        std::cout << "[SUCCESS] Number of detections: " << results.size() << "\n";

        std::set<int> found_classes = {};
        for (const auto& det : results) {
            found_classes.insert(det.classId);
        }
        std::cout << "[INFO] Total distinct classes: " << found_classes.size() << "\n";
        for (const auto& el : found_classes) {
            if ((size_t)el < labels.size())
                std::cout << el << " " << labels[el] << std::endl;
            else
                std::cout << "(!) class index out of scope: " << el << std::endl;
        }
    }

    void testDrawDetections(
        const std::string image_path,
        const std::string labels_path,
        const std::string output_path,
        bool display_result = true
    ) {
        std::cout << "[TEST] Draw detection and save to file\n";

        cv::Mat image = this->loadImage(image_path);
        if (image.empty()) return;
        std::vector<std::string> labels = this->loadLabels(labels_path);
        if (labels.size() == 0) return;

        float min_confidence = this->tracker_->config_.yolo_min_confidence;
        std::vector<yolos::Detection> results = this->tracker_->detector_->detect(image, min_confidence);
        if (results.size() == 0) {
            std::cout << "[ERROR] Empty detection results!\n";
            return;
        }

        cv::Mat output_image;
        image.copyTo(output_image);
        this->tracker_->detector_->drawDetections(output_image, results);
        cv::imwrite(output_path, output_image);
        std::cout << "[SUCCESS] Drawing complete\n";

        if (display_result) {
            std::cout << "[INFO] Display image preview - press any key to continue\n";
            cv::imshow("Output preview", output_image);
            cv::waitKey(0);
            cv::destroyWindow("Output preview");
        }
    }

    void testTiming(const std::string image_path) {
        std::cout << "[TEST] Inference timing\n";

        cv::Mat image = this->loadImage(image_path);
        if (image.empty()) return;

        std::cout << "[INFO] Running warmup inference\n";
        std::vector<yolos::Detection> results = this->tracker_->detector_->detect(image);

        this->runLoops(image, 10);
        this->runLoops(image, 100);
        this->runLoops(image, 1000);

        std::cout << "[SUCCESS] Timing complete\n";
    }

    void testFollowing() {
        std::cout << "[TEST] Following mode (simple)\n";

        Eigen::Vector3f target = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
        Eigen::Vector3f position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

        this->tracker_->current_target_ = target;
        Eigen::Vector3f goal = this->tracker_->follow_target(position);

        if (eqm(goal.x(), 0.787868f, 0.000001f) && eqm(goal.y(), 0.787868f, 0.000001f) && goal.z() == 0.0f)
            std::cout << "[SUCCESS] New goal position meets expectation" << std::endl;
        else
            std::cout << "[ERROR] Invalid goal position" << std::endl;

        std::cout << "New goal position: " << goal.x() << " " << goal.y() << " " << goal.z() << std::endl;
    }

protected:

    cv::Mat loadImage(const std::string image_path) {
        cv::Mat image = cv::imread(image_path);
        if (image.empty())
            std::cout << "[ERROR] Image could not be loaded! ('" << image_path << "')\n";
        return image;
    }

    std::vector<std::string> loadLabels(const std::string labels_path) {
        std::vector<std::string> labels = yolos::utils::getClassNames(labels_path);
        if (labels.size() == 0) 
            std::cout << "[ERROR] Labels could not be loaded! ('" << labels_path << "')\n";
        return labels;
    }

    void runLoops(const cv::Mat image, int loops) {

        double avg_time = 0.0;
        std::cout << "[INFO] Measuring " << loops << " loops\n";
        for (int l = 0; l < loops; l++) {
            std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();

            std::vector<yolos::Detection> results = this->tracker_->detector_->detect(image);

            std::chrono::duration<double> time_passed = std::chrono::system_clock::now() - time_start;
            avg_time += time_passed.count();
        }
        avg_time = avg_time / loops;
        std::cout << "[INFO] Average inference time: " << avg_time << " [s]\n";
    }

    std::unique_ptr<Tracker> tracker_;

};


} // namespace NAVIGATION_CORE


int main() {

    std::string model_path = "test_data/vision_module/models/yolo26m.onnx";
    std::string image_path = "test_data/vision_module/dataset/inputs/test_image.jpg";
    std::string labels_path = "test_data/vision_module/dataset/labels/coco.names";
    std::string output_path = "test_data/vision_module/dataset/outputs/test_image_results.jpg";

    NAVIGATION_CORE::TrackerConfig tracker_config = {};
    tracker_config.yolo_model_path = model_path;
    tracker_config.yolo_labels_path = labels_path;
    tracker_config.following_mode = true;
    tracker_config.following_distance = 0.3f;

    NAVIGATION_CORE::TrackerTestClass tracker_test_class = NAVIGATION_CORE::TrackerTestClass();
    tracker_test_class.initTracker(tracker_config);
    tracker_test_class.testReadImage(image_path, false);
    tracker_test_class.testReadLabels(labels_path);
    tracker_test_class.testInference(image_path, labels_path);
    tracker_test_class.testDrawDetections(image_path, labels_path, output_path, false);
    tracker_test_class.testTiming(image_path);
    tracker_test_class.testFollowing();
    
    return 0;
}
