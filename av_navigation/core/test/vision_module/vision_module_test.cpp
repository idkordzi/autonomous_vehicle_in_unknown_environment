#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

#include "camera_wrapper.hpp"
#include "yolo_wrapper.hpp"

#include <iostream>
#include <set>
#include <vector>
#include <string>


namespace NAVIGATION_CORE {

class YOLOWrapperTestClass : public YOLOWrapper {

public:
  
  YOLOWrapperTestClass() = default;
  ~YOLOWrapperTestClass() = default;

  void runTest() {

    std::cout << "[INFO] Running 'drone_yolo_wrapper_test': unit tests\n\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: read image from file\n";

    std::string image_path = "src/yolo/dataset/000000017627.jpg";
    cv::Mat image = cv::imread(image_path);

    if (image.empty())
      std::cout << "[ERROR] Image could not be loaded! ('" << image_path << "')\n";
    else
      std::cout << "[SUCCESS] Image loaded - image size (" << image.size[0] << "x" << image.size[1] << ")\n";

    std::cout << "Display image preview - press any key to continue\n";
    cv::imshow("Input preview", image);
    cv::waitKey(0);

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: add image\n";

    this->setInput(image);

    if (this->input_.empty())
      std::cout << "[ERROR] Could not set input image!\n";
    else
      std::cout << "[SUCCESS] Input image cached - image size (" << this->input_.size[0] << "x" << this->input_.size[1] << ")\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: run one inferance\n";

    this->run();

    std::vector<YOLO_DETECTOR::Detection> results = this->detector_->detect(this->input_);

    if (results.size() < 1)
      std::cout << "[ERROR] Could not run inference\n";
    else {
      std::cout << "[SUCCESS] Numebr of detections: " << results.size() << "\n";

      std::set<int> found_classes = {};
      for (const auto& det : results) {
        found_classes.insert(det.classId);
      }
      std::cout << "Total distinct classes: " << found_classes.size() << "\n";

      std::vector<std::string> class_names = YOLO_DETECTOR::getClassNames("src/yolo/labels/coco.names");
      for (const auto& el : found_classes) {
        std::cout << el << " " << class_names[el] <<std::endl;
      }
    }

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: draw detection and save to file\n";

    cv::Mat output_image;
    image.copyTo(output_image);
    this->detector_->drawBoundingBox(output_image, results);

    std::cout << "Display image preview - press any key to continue\n";
    cv::imshow("Output preview", output_image);
    cv::waitKey(0);

    Eigen::Vector2i target_loc = this->getTarget();
    std::cout << "Target location: (" << target_loc.x() << ", " << target_loc.y() << ")\n";

    cv::imwrite("src/yolo/dataset/000000017627_out.jpg", output_image);

    std::cout << "[INFO] Test STOP\n\n";
  }

  void runTiming() {

    std::cout << "[INFO] Running 'drone_yolo_wrapper_test': timing\n\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: inference timing\n";

    std::string image_path = "src/yolo/dataset/000000017627.jpg";
    cv::Mat image = cv::imread(image_path);
    Eigen::Vector2i target_loc = Eigen::Vector2i::Zero();

    std::cout << "Running warmup inference\n";
    this->setInput(image);
    this->run();

    int n_loops = 1000;
    double avg_time = 0.0;
    std::cout << "Running " << n_loops << " loops\n";
    for (int l = 0; l < n_loops; l++) {
      std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();

      this->setInput(image);
      this->run();
      target_loc = this->getTarget();

      std::chrono::duration<double> time_passed = std::chrono::system_clock::now() - time_start;
      avg_time += time_passed.count();
    }
    avg_time = avg_time / n_loops;
    std::cout << "Average inference time: " << avg_time << " [s]\n";

    std::cout << "[INFO] Test STOP\n\n";
  }

};


class YOLOTimingTestClass : public YOLOWrapper {

public:
  
  YOLOTimingTestClass() = default;
  ~YOLOTimingTestClass() = default;

  void runTest() {

    std::cout << "[INFO] Running: 'drone_yolo_wrapper_timing': model inference\n";
    this->runTiming(10);
    this->runTiming(100);
    this->runTiming(1000);
    std::cout << "[INFO] Test STOP\n\n";
  }

  void runTiming(int loops) {

    std::string image_path = "src/yolo/dataset/000000017627.jpg";
    cv::Mat image = cv::imread(image_path);
    Eigen::Vector2i target_loc = Eigen::Vector2i::Zero();

    std::cout << "Running warmup inference\n";
    this->setInput(image);
    this->run();

    int n_loops = loops;
    double avg_time = 0.0;
    std::cout << "Running " << n_loops << " loops\n";
    for (int l = 0; l < n_loops; l++) {
      std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();

      this->setInput(image);
      this->run();
      target_loc = this->getTarget();

      std::chrono::duration<double> time_passed = std::chrono::system_clock::now() - time_start;
      avg_time += time_passed.count();
    }
    avg_time = avg_time / n_loops;
    std::cout << "Average inference time: " << avg_time << " [s]\n";
  }

};
    
} // namespace NAVIGATION_CORE


int main() {

  NAVIGATION_CORE::YOLOWrapperTestClass yolo_wrapper_test_class = NAVIGATION_CORE::YOLOWrapperTestClass();
  yolo_wrapper_test_class.runTest();
  yolo_wrapper_test_class.runTiming();

  NAVIGATION_CORE::YOLOTimingTestClass yolo_timing_test_class = NAVIGATION_CORE::YOLOTimingTestClass();
  yolo_timing_test_class.runTest();

  return 0;
}