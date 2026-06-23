#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

#include "local_planner.hpp"

#include <iostream>
#include <set>
#include <vector>
#include <string>


namespace NAVIGATION_CORE {

inline bool eqmarg(float a, float b, float m) {
  return std::abs(a-b) <= m;
}

class PolarHistogramTestClass {

public:
  PolarHistogramTestClass() = default;
  ~PolarHistogramTestClass() = default;

  void runTest() {

    std::cout << "[INFO] Running 'drone_polar_histogram_test': unit tests\n\n";

    PolarHistogram histogram = PolarHistogram(6); // cell size of 6 degrees

    if (histogram.isEmpty())
      std::cout << "[SUCCESS] Initialization with (6) [deg] cell size\n";
    else
      std::cout << "[ERROR] Histogram not empty on initialization\n";
    
    histogram.setDistance(0, 0, 1.0f);

    if (eqmarg(histogram.getDistance(0, 0), 1.0f, 0.0f))
      std::cout << "[SUCCESS] Successfully wrote and read from histogram at (0, 0) [deg] (elevetion, azimuth)\n";
    else
      std::cout << "[ERROR] When reading from cell at (0, 0) [deg] (elevetion, azimuth): '" << histogram.getDistance(0, 0) << "' =/= 1.0\n";

    Eigen::Vector2i h_idx = convertAngleToHistogramIndex(61.0f, 121.0f, 6);
    histogram.setDistance(h_idx.y(), h_idx.x(), 1.0f);

    if (eqmarg(histogram.getDistance(h_idx.y(), h_idx.x()), 1.0f, 0.0f))
      std::cout << "[SUCCESS] Successfully wrote and read from histogram at (61, 121) [deg] (elevetion, azimuth)\n";
    else
      std::cout << "[ERROR] When reading from cell at (61, 121) [deg] (elevetion, azimuth): '" << histogram.getDistance(h_idx.y(), h_idx.x())
                << "' =/= 1.0\n";

    std::cout << "[INFO] Test STOP\n\n";
  }
};

class LocalPlannerTestClass : public LocalPlanner {

public:
  LocalPlannerTestClass() = default;
  ~LocalPlannerTestClass() = default;

  void runTest() {

    std::cout << "[INFO] Running 'drone_local_planner_test': unit tests\n\n";

    Eigen::Vector3f position    = Eigen::Vector3f::Zero();
    Eigen::Vector3f orientation = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity    = Eigen::Vector3f::Zero();

    Eigen::Vector3f goal = Eigen::Vector3f::Zero();

    PointXYZ point_xyz = PointXYZ();
    PointCloud<PointXYZ> point_cloud = PointCloud<PointXYZ>();

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: 'add new goal'\n";

    goal = Eigen::Vector3f(5.0f, 0.0f, 1.0f);
    this->setGoal(goal);

    if (this->goal_updated_) {
      if (eqmarg(this->goal_.x(), 5.0f, 0.0f) && 
          eqmarg(this->goal_.y(), 0.0f, 0.0f) && 
          eqmarg(this->goal_.z(), 1.0f, 0.0f))
        std::cout << "[SUCCESS] New goal updated\n";
      else
        std::cout << "[ERROR] Mismatch on new goal: (" << this->goal_.x() << ", " << this->goal_.y() << ", " << this->goal_.z() 
                  << ") =/= (5.0, 0.0, 1.0)\n";
      
      if (eqmarg(this->goal_pos_.x(), 1.127f, 0.001f) &&
          eqmarg(this->goal_pos_.y(),   0.0f, 0.001f) &&
          eqmarg(this->goal_pos_.z(),   2.0f, 0.001f))
        std::cout << "[SUCCESS] New goal position updated\n";
      else
        std::cout << "[ERROR] Mismatch on new goal position: (" << this->goal_pos_.x() << ", " << this->goal_pos_.y() << ", " 
                  << this->goal_pos_.z() << ") =/= (1.127.., 0.0, 2.0)\n";
    }
    else
      std::cout << "[ERROR] Could not add new goal\n";
    
    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: 'add new goal within ignorance margin'\n";

    goal = Eigen::Vector3f(5.01, 0.01, 1.01);
    this->setGoal(goal);

    if (this->goal_updated_) {
      std::cout << "[ERROR] New goal updated despite being inside ignorance margin\n";
    }
    else
      std::cout << "[SUCCESS] New goal ingored (point within ignorance margin)\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: 'add new goal with preserving previuos goal\n";

    goal = Eigen::Vector3f(6.0, 1.0, 2.0);
    this->setGoal(goal);

    if (this->goal_updated_) {
      Eigen::Vector3f prev_goal = this->prev_goal_array_[1];
      if (eqmarg(this->goal_.x(), 6.0f, 0.0f) && 
          eqmarg(this->goal_.y(), 1.0f, 0.0f) && 
          eqmarg(this->goal_.z(), 2.0f, 0.0f))
        std::cout << "[SUCCESS] New goal updated\n";
      else
        std::cout << "[ERROR] Mismatch on new goal: (" << this->goal_.x() << ", " << this->goal_.y() << ", " << this->goal_.z() 
                  << ") =/= (6.0, 1.0, 2.0)\n";
      if (eqmarg(prev_goal.x(), 5.0f, 0.0f) && 
          eqmarg(prev_goal.y(), 0.0f, 0.0f) && 
          eqmarg(prev_goal.z(), 1.0f, 0.0f))
        std::cout << "[SUCCESS] Previous goal preserved\n";
      else
        std::cout << "[ERROR] Mismatch on previous goal: ("<< prev_goal.x() << ", " << prev_goal.y() << ", " << prev_goal.z() 
                  << ") =/= (5.0, 0.0, 1.0)\n";
    }
    else
      std::cout << "[ERROR] Could not add new goal\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: 'update state'\n";

    position    = Eigen::Vector3f::Zero();
    orientation = Eigen::Vector3f::Zero();
    velocity    = Eigen::Vector3f::Zero();

    this->state_updated_ = false;
    this->setState(position, orientation, velocity);

    bool no_mismatch = true;
    if (this->state_updated_) {
      if (!eqmarg(this->position_.x(), 0.0f, 0.0f) ||
          !eqmarg(this->position_.y(), 0.0f, 0.0f) ||
          !eqmarg(this->position_.z(), 0.0f, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: position\n";
      }
      if (!eqmarg(this->orientation_.x(), 0.0f, 0.0f) ||
          !eqmarg(this->orientation_.y(), 0.0f, 0.0f) ||
          !eqmarg(this->orientation_.z(), 0.0f, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: orientation\n";
      }
      if (!eqmarg(this->lin_velocity_.x(), 0.0f, 0.0f) ||
          !eqmarg(this->lin_velocity_.y(), 0.0f, 0.0f) ||
          !eqmarg(this->lin_velocity_.z(), 0.0f, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: velocity\n";
      }
      if (!eqmarg(this->fov_.yaw_deg, 0.0f, 0.0f) || 
          !eqmarg(this->fov_.pitch_deg, 0.0f, 0.0f) || 
          !eqmarg(this->fov_.h_fov_deg, this->config_.camera_fov_h, 0.0f) || 
          !eqmarg(this->fov_.v_fov_deg, this->config_.camera_fov_v, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: FOV\n";
      }
      if (no_mismatch)
        std::cout << "[SUCCESS] State updated\n";
    }
    else
      std::cout << "[ERROR] Could not update state\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: 'update state - non-zero input'\n";

    position    = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    orientation = Eigen::Vector3f(0.5f, 1.0f, 1.5f);
    velocity    = Eigen::Vector3f(0.5f, 0.5f, 0.5f);

    this->state_updated_ = false;
    this->setState(position, orientation, velocity);

    no_mismatch = true;
    if (this->state_updated_) {
      if (!eqmarg(this->position_.x(), 1.0f, 0.0f) ||
          !eqmarg(this->position_.y(), 2.0f, 0.0f) ||
          !eqmarg(this->position_.z(), 3.0f, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: position\n";
      }
      if (!eqmarg(this->orientation_.x(), 0.5f, 0.0f) ||
          !eqmarg(this->orientation_.y(), 1.0f, 0.0f) ||
          !eqmarg(this->orientation_.z(), 1.5f, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: orientation\n";
      }
      if (!eqmarg(this->lin_velocity_.x(), 0.5f, 0.0f) ||
          !eqmarg(this->lin_velocity_.y(), 0.5f, 0.0f) ||
          !eqmarg(this->lin_velocity_.z(), 0.5f, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: velocity\n";
      }
      if (!eqmarg(this->fov_.yaw_deg, 1.5f * RAD_TO_DEG, 0.001f) || 
          !eqmarg(this->fov_.pitch_deg, 1.0f * RAD_TO_DEG, 0.001f) || 
          !eqmarg(this->fov_.h_fov_deg, this->config_.camera_fov_h, 0.0f) || 
          !eqmarg(this->fov_.v_fov_deg, this->config_.camera_fov_v, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: FOV\n";
      }
      if (no_mismatch)
        std::cout << "[SUCCESS] State updated\n";
    }
    else
      std::cout << "[ERROR] Could not update state\n";
    
    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: 'add new point cloud\n";

    position    = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
    orientation = Eigen::Vector3f(0.0f, 0.0f, 1.57079f);
    velocity    = Eigen::Vector3f::Zero();

    this->setState(position, orientation, velocity);

    point_xyz = PointXYZ(1.0, -1.0, 1.0);
    point_cloud.clear();
    point_cloud.push_back(point_xyz);

    this->setPointCloud(point_cloud);

    if (this->cloud_updated_) {
      if (this->cloud_cache_.size() != 1)
        std::cout << "[ERROR] Incorrect point cloud size: '" << this->cloud_cache_.size() << "' =/= 1\n";
      else if(!eqmarg(this->cloud_cache_[0].x, 2.0f, 0.001f) || 
              !eqmarg(this->cloud_cache_[0].y, 2.0f, 0.001f) || 
              !eqmarg(this->cloud_cache_[0].z, 1.0f, 0.001f))
        std::cout << "[ERROR] Mismatch on 3D point in added point cloud: (" << this->cloud_cache_[0].x << ", " 
                  << this->cloud_cache_[0].y << ", " << this->cloud_cache_[0].z << ") =/= (2.0, 2.0, 1.0)\n";
      else
        std::cout << "[SUCCESS] New point cloud updated\n";
    }
    else
      std::cout << "[ERROR] Could not add new point cloud\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: processing point cloud\n";

    position    = Eigen::Vector3f::Zero();
    orientation = Eigen::Vector3f::Zero();
    velocity    = Eigen::Vector3f::Zero();

    this->setState(position, orientation, velocity);

    this->fov_.h_fov_deg = 90.0f;
    this->fov_.v_fov_deg = 90.0f;
    
    point_cloud.clear();
    point_cloud.push_back(PointXYZ(0.5, 0.5, 0.5)); // inside FOV - will be saved
    // point_cloud.push_back(PointXYZ(0.5, 0.5, 1.0)); // outside FOV - will be discarded
    point_cloud.push_back(PointXYZ(15.0, 0.0, 0.0)); // outside sensor range - will be discarded
    this->setPointCloud(point_cloud);

    this->processPointCloud();

    Eigen::Vector2i index = convertPolarToHistogramIndex(convertCartesianToPolar(toEigen(point_cloud[0])), this->histogram_.getAlpha());

    if (!this->histogram_.isEmpty()) {
      if (eqmarg(this->histogram_.getDistance(index.y(), index.x()), 0.866f, 0.001f))
        std::cout << "[SUCCESS] Point cloud processed w/o issues\nAt (" << index.y() << ", " << index.x() << ") distance: "
                  << this->histogram_.getDistance(index.y(), index.x()) << " [m]\n";
      else
        std::cout << "[ERROR] Incorrect distance at (" << index.y() <<  ", " << index.x() << ") [deg] (elevation, azimuth): ("
                  << this->histogram_.getDistance(index.y(), index.x()) << ")\n";
    }
    else
      std::cout << "[ERROR] Histogram has not been updated (histogram empty)\n";
    
    // this->histogram_.setCell(index.y(), index.x(), 0.0f, 0.0f);
    // if (this->histogram_.isEmpty())
    //   std::cout << "[SUCCESS]\n";
    // else
    //   std::cout << "[ERROR]\n";

    // for (int e = 0 ; e < this->histogram_.getElevRes(); e++) {
    //   for (int z = 0 ; z < this->histogram_.getAzimRes(); z++)
    //     std::cout << this->histogram_.getDistance(e,z) << " ";
    //   std::cout << "\n";
    // }

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: empty run\n";

    position    = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    orientation = Eigen::Vector3f::Zero();
    velocity    = Eigen::Vector3f::Zero();

    this->goal_ = Eigen::Vector3f::Zero();
    this->next_ = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    this->histogram_.clear();
    
    this->setState(position, orientation, velocity);

    goal = Eigen::Vector3f(1.0f, 0.0f, 0.0f); // in camera frame
    this->setGoal(goal);

    point_cloud.clear();
    this->setPointCloud(point_cloud);

    this->run();

    Eigen::Vector3f next_pos = this->getNext();

    if (eqmarg((this->goal_ - this->goal_pos_).norm(), this->config_.goal_min_dist, 0.001f) && 
        eqmarg(std::abs(this->goal_.z() - this->goal_pos_.z()), this->config_.goal_min_alt_diff, 0.001f))
      std::cout << "[SUCCESS] Empty run w/o issues\n";
    else
      std::cout << "[ERROR] calculated next goal position does not fulfill requirements\n";
    
    std::cout << "Print target position: (" << this->goal_.x() << ", " << this->goal_.y() << ", " << this->goal_.z() << ")\n";
    std::cout << "Print goal position: (" << this->goal_pos_.x() << ", " << this->goal_pos_.y() << ", " << this->goal_pos_.z() 
              << "), distance: (" << (this->goal_ - this->goal_pos_).norm() << ")\n";
    std::cout << "Print next position: (" << next_pos.x() << ", " << next_pos.y() << ", " << next_pos.z() << "\n";
    
    std::cout << "[INFO] Test STOP\n\n";
  }

  void runTiming() {

    std::cout << "[INFO] Running 'drone_local_planner_test': timing\n\n";

    Eigen::Vector3f position    = Eigen::Vector3f::Zero();
    Eigen::Vector3f orientation = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity    = Eigen::Vector3f::Zero();

    Eigen::Vector3f goal = Eigen::Vector3f::Zero();

    PointCloud<PointXYZ> point_cloud = PointCloud<PointXYZ>();

    double avg_time = 0.0;
    double max_time = 0.0;
    double min_time = 1e3;
    double passed = 0.0;
    std::chrono::duration<double> time_passed;
    int n_loops = 0;

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: timing adding point cloud\n";

    position    = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    orientation = Eigen::Vector3f::Zero();
    velocity    = Eigen::Vector3f::Zero();

    this->next_ = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    this->histogram_.clear();

    goal = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

    point_cloud.clear();

    int c_width = 640;
    int c_height = 360;

    double step_w = 10.0 / (c_width / 2);
    double step_h = 5.0 / (c_height / 2);
    double step_d = 1.0 / (c_width / 2);

    Eigen::Vector3f sp = Eigen::Vector3f(step_d, step_w, 0.0);
    for (int i = 0; i < c_height; i++) {
      Eigen::Vector3f np = Eigen::Vector3f(5.0, -10.0, -5.0);
      for (int j = 0; j < c_width; j++) {
        point_cloud.push_back(PointXYZ(np.x(), np.y(), np.z()));
        np += sp;
      }
      np += Eigen::Vector3f(0.0, 0.0, step_h);
    }

    this->reset();

    avg_time = 0.0;
    max_time = 0.0;
    min_time = 1e3;
    passed = 0.0;
    n_loops = 100;
    std::cout << "Running " << n_loops << " loops\n";
    for (int l = 0; l < n_loops; l++) {

      std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();

      this->setPointCloud(point_cloud);

      time_passed = std::chrono::system_clock::now() - time_start;
      passed = time_passed.count();
      avg_time += passed;
      max_time = passed > max_time ? passed : max_time;
      min_time = passed < min_time ? passed : min_time;
    }
    avg_time = avg_time / n_loops;
    std::cout << "Processing time: average: " << avg_time << "; max: " << max_time << "; min: " << min_time << " [s]\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: timing processing point cloud\n";

    this->histogram_.clear();

    this->setState(position, orientation, velocity);
    this->setPointCloud(point_cloud);
    this->processPointCloud();

    avg_time = 0.0;
    max_time = 0.0;
    min_time = 1e3;
    passed = 0.0;
    n_loops = 100;
    std::cout << "Running " << n_loops << " loops\n";
    for (int l = 0; l < n_loops; l++) {

      this->cloud_updated_ = true;

      std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();

      this->processPointCloud();

      time_passed = std::chrono::system_clock::now() - time_start;
      passed = time_passed.count();
      avg_time += passed;
      max_time = passed > max_time ? passed : max_time;
      min_time = passed < min_time ? passed : min_time;
    }
    avg_time = avg_time / n_loops;
    std::cout << "Processing time: average: " << avg_time << "; max: " << max_time << "; min: " << min_time << " [s]\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: timing plannig route\n";

    this->histogram_.clear();

    this->setState(position, orientation, velocity);
    this->setPointCloud(point_cloud);
    this->processPointCloud();

    avg_time = 0.0;
    max_time = 0.0;
    min_time = 1e3;
    passed = 0.0;
    n_loops = 100;
    std::cout << "Running " << n_loops << " loops\n";
    for (int l = 0; l < n_loops; l++) {

      this->cloud_updated_ = true;

      std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();

      this->planNext();

      time_passed = std::chrono::system_clock::now() - time_start;
      passed = time_passed.count();
      avg_time += passed;
      max_time = passed > max_time ? passed : max_time;
      min_time = passed < min_time ? passed : min_time;
    }
    avg_time = avg_time / n_loops;
    std::cout << "Processing time: average: " << avg_time << "; max: " << max_time << "; min: " << min_time << " [s]\n";

    std::cout << "[INFO] Test STOP\n\n";
  }
};


class CloudAccelTestClass : public LocalPlanner {

public:
  CloudAccelTestClass() = default;
  ~CloudAccelTestClass() = default;

  void runTest() {

    std::cout << "[INFO] Running: 'drone_local_planner_timing': processing point cloud\n";
    this->runTiming(10);
    this->runTiming(100);
    this->runTiming(1000);
    std::cout << "[INFO] Test STOP\n\n";
  }

  void runTiming(int loops) {

    Eigen::Vector3f position    = Eigen::Vector3f::Zero();
    Eigen::Vector3f orientation = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity    = Eigen::Vector3f::Zero();

    Eigen::Vector3f goal = Eigen::Vector3f::Zero();

    PointCloud<PointXYZ> point_cloud = PointCloud<PointXYZ>();

    double avg_time = 0.0;
    double max_time = 0.0;
    double min_time = 1e3;
    double passed = 0.0;
    std::chrono::duration<double> time_passed;
    int n_loops = 0;

    // **************** //

    position    = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    orientation = Eigen::Vector3f::Zero();
    velocity    = Eigen::Vector3f::Zero();

    goal = Eigen::Vector3f(100.0f, 0.0f, 1.0f);

    int c_width = 640;
    int c_height = 360;

    double step_w = 10.0 / (c_width / 2);
    double step_h = 5.0 / (c_height / 2);
    double step_d = 1.0 / (c_width / 2);

    point_cloud.clear();
    Eigen::Vector3f sp = Eigen::Vector3f(step_d, step_w, 0.0);
    for (int i = 0; i < c_height; i++) {
      Eigen::Vector3f np = Eigen::Vector3f(5.0, -10.0, -5.0);
      for (int j = 0; j < c_width; j++) {
        point_cloud.push_back(PointXYZ(np.x(), np.y(), np.z()));
        np += sp;
      }
      np += Eigen::Vector3f(0.0, 0.0, step_h);
    }

    // **************** //

    this->reset();

    this->setState(position, orientation, velocity);
    this->setGoal(goal);
    this->setPointCloud(point_cloud);
    this->processPointCloud();

    avg_time = 0.0;
    max_time = 0.0;
    min_time = 1e3;
    passed = 0.0;
    n_loops = loops;
    std::cout << "Running " << n_loops << " loops\n";
    for (int l = 0; l < n_loops; l++) {

      this->cloud_updated_ = true;

      std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();

      this->processPointCloud();

      time_passed = std::chrono::system_clock::now() - time_start;
      passed = time_passed.count();
      avg_time += passed;
      max_time = passed > max_time ? passed : max_time;
      min_time = passed < min_time ? passed : min_time;
    }
    avg_time = avg_time / n_loops;
    std::cout << "Processing time: average: " << avg_time << "; max: " << max_time << "; min: " << min_time << " [s]\n";
  }

};

} // namespace NAVIGATION_CORE


int main() {

  NAVIGATION_CORE::PolarHistogramTestClass polar_histogram_test_class = NAVIGATION_CORE::PolarHistogramTestClass();
  polar_histogram_test_class.runTest();

  NAVIGATION_CORE::LocalPlannerTestClass local_planner_test_class = NAVIGATION_CORE::LocalPlannerTestClass();
  local_planner_test_class.runTest();
  local_planner_test_class.runTiming();

  NAVIGATION_CORE::CloudAccelTestClass cloud_accel_test_class = NAVIGATION_CORE::CloudAccelTestClass();
  cloud_accel_test_class.runTest();

  return 0;
}