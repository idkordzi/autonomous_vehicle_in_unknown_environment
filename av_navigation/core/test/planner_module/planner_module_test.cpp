#include <iostream>
#include <memory>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Dense"

#include "local_planner.hpp"


namespace NAVIGATION_CORE {


inline bool eqm(float a, float b, float m) {
    return std::abs(a-b) <= m;
}


class PolarHistogramTestClass {

public:

    PolarHistogramTestClass() = default;
    ~PolarHistogramTestClass() = default;

    void initHistogram(int alpha) {
        std::cout << "[INFO] Initializing HISTOGRAM class with ALPHA=" << alpha << std::endl;
        this->histogram_ = std::make_unique<PolarHistogram>(alpha);

        if (!this->histogram_->isEmpty())
            std::cout << "[ERROR] Histogram not empty on initialization\n";
    }

    void testClear() {
        std::cout << "[TEST] Histogram clear\n";

        this->histogram_->clear();

        if (this->histogram_->isEmpty())
            std::cout << "[SUCCESS] Clear: OK\n";
        else
            std::cout << "[ERROR] Histogram not empty after clear\n";
    }

    void testUpdateCells() {
        std::cout << "[TEST] Updating single cell at (0, 0) deg\n";

        this->histogram_->setDistance(0, 0, 1.0f);

        if (eqm(this->histogram_->getDistance(0, 0), 1.0f, 0.0f))
            std::cout << "[SUCCESS] Successfully updated histogram\n";
        else
            std::cout << "[ERROR] Invalid cell value: " << this->histogram_->getDistance(0, 0) << " =/= 1.0\n";

        std::cout << "[TEST] Updating single cell at (61, 121) deg\n";

        Eigen::Vector2i histogram_idx = convertAngleToHistogramIndex(61.0f, 121.0f, 6);
        this->histogram_->setDistance(histogram_idx.y(), histogram_idx.x(), 1.0f);

        if (eqm(this->histogram_->getDistance(histogram_idx.y(), histogram_idx.x()), 1.0f, 0.0f))
            std::cout << "[SUCCESS] Successfully updated histogram\n";
        else
            std::cout << "[ERROR] nvalid cell value: " << this->histogram_->getDistance(histogram_idx.y(), histogram_idx.x()) << " =/= 1.0\n";
    }

protected:

    std::unique_ptr<NAVIGATION_CORE::PolarHistogram> histogram_;
};

class PlannerTestClass {

public:
    PlannerTestClass() = default;
    ~PlannerTestClass() = default;

    void initPlanner(LocalPlannerConfig config) {
        std::cout << "[INFO] Initializing PLANNER class\n";
        this->planner_ = std::make_unique<LocalPlanner>(config);
        this->planner_->reset();
    }

    void testAddNewGoal() {
        std::cout << "[TEST] Add new goal\n";
        this->planner_->reset();

        Eigen::Vector3f target = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
        this->planner_->setTarget(target);

        if (this->planner_->goal_updated_) {
            if (
                eqm(this->planner_->goal_.x(), 1.0f, 0.0f)
                && eqm(this->planner_->goal_.y(), 1.0f, 0.0f)
                && eqm(this->planner_->goal_.z(), 0.0f, 0.0f)
            )
                std::cout << "[SUCCESS] New goal updated\n";
            else
                std::cout << "[ERROR] Mismatch on new goal: ("
                    << this->planner_->goal_.x() << ", "
                    << this->planner_->goal_.y() << ", "
                    << this->planner_->goal_.z() << ") =/= (1.0, 1.0, 0.0)\n";
            
            if (
                eqm(this->planner_->goal_pos_.x(), 0.292f, 0.001f)
                && eqm(this->planner_->goal_pos_.y(), 0.292f, 0.001f)
                && eqm(this->planner_->goal_pos_.z(), 0.0f, 0.001f)
            )
                std::cout << "[SUCCESS] New goal position updated\n";
            else
                std::cout << "[ERROR] Mismatch on new goal position: ("
                    << this->planner_->goal_pos_.x() << ", "
                    << this->planner_->goal_pos_.y() << ", "
                    << this->planner_->goal_pos_.z() << ") =/= (0.292.., 0.292, 0.0)\n";
        }
        else
            std::cout << "[ERROR] Could not add new goal\n";
    }

    void testAddNewGoalIgnore() {
        std::cout << "[TEST] Add new goal within ignorance margin\n";
        this->planner_->reset();

        Eigen::Vector3f target = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
        this->planner_->setTarget(target);
        this->planner_->goal_updated_ = false;

        target = Eigen::Vector3f(1.001, 1.001, 0.00);
        this->planner_->setTarget(target);

        if (!this->planner_->goal_updated_)
            std::cout << "[SUCCESS] New goal ingored (point within ignorance margin)\n";
        else
            std::cout << "[ERROR] New goal updated despite being inside ignorance margin\n";
    }

    void testUpdateState() {
        std::cout << "[TEST] Update state\n";
        this->planner_->reset();

        Eigen::Vector3f position = Eigen::Vector3f::Zero();
        Eigen::Vector4f orientation = Eigen::Vector4f::Zero();
        Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

        this->planner_->setState(position, orientation, velocity);

        bool no_mismatch = true;
        if (this->planner_->state_updated_) {
            if (
                !eqm(this->planner_->position_.x(), 0.0f, 0.0f)
                || !eqm(this->planner_->position_.y(), 0.0f, 0.0f)
                || !eqm(this->planner_->position_.z(), 0.0f, 0.0f)
            ) {
                no_mismatch = false;
                std::cout << "[ERROR] Mismatch on position\n";
            }
            if (
                !eqm(this->planner_->orientation_.x(), 0.0f, 0.0f)
                || !eqm(this->planner_->orientation_.y(), 0.0f, 0.0f)
                || !eqm(this->planner_->orientation_.z(), 0.0f, 0.0f)
                || !eqm(this->planner_->orientation_.w(), 0.0f, 0.0f)
            ) {
                no_mismatch = false;
                std::cout << "[ERROR] Mismatch on orientation\n";
            }
            if (
                !eqm(this->planner_->velocity_.x(), 0.0f, 0.0f)
                || !eqm(this->planner_->velocity_.y(), 0.0f, 0.0f)
                || !eqm(this->planner_->velocity_.z(), 0.0f, 0.0f)
            ) {
                no_mismatch = false;
                std::cout << "[ERROR] Mismatch on velocity\n";
            }
            if (
                !eqm(this->planner_->fov_.yaw_deg, 0.0f, 0.0f)
                || !eqm(this->planner_->fov_.pitch_deg, 0.0f, 0.0f)
                || !eqm(this->planner_->fov_.h_fov_deg, this->planner_->config_.camera_fov_h, 0.0f)
                || !eqm(this->planner_->fov_.v_fov_deg, this->planner_->config_.camera_fov_v, 0.0f)
            ) {
                no_mismatch = false;
                std::cout << "[ERROR] Mismatch on FOV\n";
            }
            if (no_mismatch)
                std::cout << "[SUCCESS] State updated\n";
        }
        else
            std::cout << "[ERROR] Could not update state\n";
    }

    void testUpdateStateNonZero() {
        std::cout << "[TEST] Update state with non-zero values\n";
        this->planner_->reset();

        Eigen::Vector3f position = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
        Eigen::Vector4f orientation = Eigen::Vector4f(0.0f, 0.0f, 0.3826834f, 0.9238796f); // 45 deg (0.785398 rad) to right
        Eigen::Vector3f velocity = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

        this->planner_->state_updated_ = false;
        this->planner_->setState(position, orientation, velocity);

        bool no_mismatch = true;
        if (this->planner_->state_updated_) {
            if (
                !eqm(this->planner_->position_.x(), 1.0f, 0.0f)
                || !eqm(this->planner_->position_.y(), 1.0f, 0.0f)
                || !eqm(this->planner_->position_.z(), 0.0f, 0.0f)
            ) {
                no_mismatch = false;
                std::cout << "[ERROR] Mismatch on position: ("
                    << this->planner_->position_.x() << ", "
                    << this->planner_->position_.y() << ", "
                    << this->planner_->position_.z() << ") =/= (1.0, 1.0, 0.0)\n";
            }
            if (
                !eqm(this->planner_->orientation_.x(), 0.0f, 0.0f)
                || !eqm(this->planner_->orientation_.y(), 0.0f, 0.0f)
                || !eqm(this->planner_->orientation_.z(), 0.382f, 0.001f)
                || !eqm(this->planner_->orientation_.w(), 0.923f, 0.001f)
            ) {
                no_mismatch = false;
                std::cout << "[ERROR] Mismatch on orientation: ("
                    << this->planner_->orientation_.x() << ", "
                    << this->planner_->orientation_.y() << ", "
                    << this->planner_->orientation_.z() << ", "
                    << this->planner_->orientation_.w() << ") =/= (0.0, 0.0, 0.382.., 0.923..)\n";
            }
            if (
                !eqm(this->planner_->velocity_.x(), 1.0f, 0.0f)
                || !eqm(this->planner_->velocity_.y(), 0.0f, 0.0f)
                || !eqm(this->planner_->velocity_.z(), 0.0f, 0.0f)
            ) {
                no_mismatch = false;
                std::cout << "[ERROR] Mismatch on velocity: ("
                    << this->planner_->velocity_.x() << ", "
                    << this->planner_->velocity_.y() << ", "
                    << this->planner_->velocity_.z() << ") =/= (1.0, 0.0, 0.0)\n";
            }
            if (
                !eqm(this->planner_->fov_.yaw_deg, 45.0f, 0.001f)
                || !eqm(this->planner_->fov_.pitch_deg, 0.0f, 0.0f)
                || !eqm(this->planner_->fov_.h_fov_deg, this->planner_->config_.camera_fov_h, 0.0f)
                || !eqm(this->planner_->fov_.v_fov_deg, this->planner_->config_.camera_fov_v, 0.0f)
            ) {
                no_mismatch = false;
                std::cout << "[ERROR] Mismatch on FOV: ("
                    << this->planner_->fov_.yaw_deg << ", "
                    << this->planner_->fov_.pitch_deg << ") x "
                    << this->planner_->fov_.h_fov_deg << ", "
                    << this->planner_->fov_.v_fov_deg << ") =/= (45.0, 0.0) x ("
                    << this->planner_->config_.camera_fov_h << ", " << this->planner_->config_.camera_fov_v << ")\n";
            }
            if (no_mismatch)
                std::cout << "[SUCCESS] State updated\n";
        }
        else
            std::cout << "[ERROR] Could not update state\n";
    }

    void testAddNewPointCloud() {
        std::cout << "[TEST] Add new point cloud\n";
        this->planner_->reset();

        PointCloud<PointXYZ> point_cloud = PointCloud<PointXYZ>();
        point_cloud.clear();
        point_cloud.push_back(PointXYZ(1.0, 1.0, 1.0));

        this->planner_->setPointCloud(point_cloud);

        if (this->planner_->cloud_updated_) {
            if (this->planner_->cloud_cache_.size() != 1)
                std::cout << "[ERROR] Incorrect point cloud size: '" << this->planner_->cloud_cache_.size() << "' =/= 1\n";
            else if(
                !eqm(this->planner_->cloud_cache_[0].x, 1.0f, 0.001f)
                || !eqm(this->planner_->cloud_cache_[0].y, 1.0f, 0.001f)
                || !eqm(this->planner_->cloud_cache_[0].z, 1.0f, 0.001f)
            )
                std::cout << "[ERROR] Mismatch on 3D point in added point cloud: ("
                    << this->planner_->cloud_cache_[0].x << ", " 
                    << this->planner_->cloud_cache_[0].y << ", "
                    << this->planner_->cloud_cache_[0].z << ") =/= (1.0, 1.0, 1.0)\n";
            else
                std::cout << "[SUCCESS] New point cloud updated\n";
        }
        else
            std::cout << "[ERROR] Could not add new point cloud\n";
    }

    void testProcessPointCloud() {
        std::cout << "[TEST] Process point cloud\n";
        this->planner_->reset();

        Eigen::Vector3f position = Eigen::Vector3f::Zero();
        Eigen::Vector4f orientation = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
        Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

        PointCloud<PointXYZ> point_cloud = PointCloud<PointXYZ>();
        point_cloud.clear();
        point_cloud.push_back(PointXYZ(0.5, 0.5, 0.5)); // inside FOV - will be saved
        // point_cloud.push_back(PointXYZ(20.0, 0.0, 0.0)); // outside sensor range - will be discarded

        this->planner_->setState(position, orientation, velocity);
        this->planner_->fov_.h_fov_deg = 90.0f;
        this->planner_->fov_.v_fov_deg = 90.0f;
        this->planner_->setPointCloud(point_cloud);
        this->planner_->processPointCloud_();

        Eigen::Vector2i histogram_idx = convertPolarToHistogramIndex(
            convertCartesianToPolar(toEigen(point_cloud[0])), this->planner_->histogram_.getAlpha()
        );

        if (!this->planner_->histogram_.isEmpty()) {
            if (eqm(this->planner_->histogram_.getDistance(histogram_idx.y(), histogram_idx.x()), 0.866f, 0.001f))
                std::cout << "[SUCCESS] Point cloud processed w/o issues\n"
                    << "cell at (" << histogram_idx.y() << ", " << histogram_idx.x() << ") distance: "
                    << this->planner_->histogram_.getDistance(histogram_idx.y(), histogram_idx.x()) << " [m]\n";
            else
                std::cout << "[ERROR] Incorrect distance at ("
                    << histogram_idx.y() <<  ", " << histogram_idx.x() << ") [deg] (elevation, azimuth): ("
                    << this->planner_->histogram_.getDistance(histogram_idx.y(), histogram_idx.x()) << ")\n";
        }
        else
            std::cout << "[ERROR] Histogram has not been updated (histogram empty)\n";
    }

    void testEmptyRun() {
        std::cout << "[TEST] Empty run\n";
        this->planner_->reset();

        Eigen::Vector3f position = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
        Eigen::Vector4f orientation = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
        Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

        Eigen::Vector3f target = Eigen::Vector3f(3.0f, 0.0f, 0.0f);

        PointCloud<PointXYZ> point_cloud = PointCloud<PointXYZ>();
        point_cloud.clear();

        this->planner_->goal_ = Eigen::Vector3f::Zero();
        this->planner_->next_ = Eigen::Vector3f::Zero();
        this->planner_->histogram_.clear();
    
        this->planner_->setState(position, orientation, velocity);
        this->planner_->setTarget(target);
        this->planner_->setPointCloud(point_cloud);
        this->planner_->run();

        Eigen::Vector3f next_pos = this->planner_->getNext();

        if (eqm((this->planner_->goal_ - this->planner_->goal_pos_).norm(), this->planner_->config_.goal_min_dist, 0.001f))
            std::cout << "[SUCCESS] Empty run w/o issues\n";
        else
            std::cout << "[ERROR] Calculated next goal position does not fulfill requirements\n";
        
        std::cout << "Print target position: ("
            << this->planner_->goal_.x() << ", "
            << this->planner_->goal_.y() << ", "
            << this->planner_->goal_.z() << ")\n";
        std::cout << "Print goal position: ("
            << this->planner_->goal_pos_.x() << ", "
            << this->planner_->goal_pos_.y() << ", "
            << this->planner_->goal_pos_.z() 
            << "), distance: ("
            << (this->planner_->goal_ - this->planner_->goal_pos_).norm() << ")\n";
        std::cout << "Print next position: ("
            << next_pos.x() << ", "
            << next_pos.y() << ", "
            << next_pos.z() << "\n";
    }

    void testTimingAddingPointCloud() {
        std::cout << "[TEST] Timing test for adding point cloud\n";
        this->planner_->reset();

        PointCloud<PointXYZ> point_cloud = this->generatePointCloud();

        double avg_time = 0.0;
        double max_time = 0.0;
        double min_time = 1e3;
        double passed = 0.0;
        int n_loops = 100;
        std::chrono::duration<double> time_passed;
        std::cout << "Running " << n_loops << " loops\n";
        for (int l = 0; l < n_loops; l++) {
            std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();
            this->planner_->setPointCloud(point_cloud);
            time_passed = std::chrono::system_clock::now() - time_start;

            passed = time_passed.count();
            avg_time += passed;
            max_time = passed > max_time ? passed : max_time;
            min_time = passed < min_time ? passed : min_time;
        }
        avg_time = avg_time / n_loops;
        std::cout << "Processing time: average: " << avg_time << "; max: " << max_time << "; min: " << min_time << " [s]\n";

    }

    void testTimingProcessPointCloud() {
        std::cout << "[TEST] Timing test for processing point cloud\n";
        this->planner_->reset();

        Eigen::Vector3f position = Eigen::Vector3f::Zero();
        Eigen::Vector4f orientation = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
        Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

        PointCloud<PointXYZ> point_cloud = this->generatePointCloud();

        this->planner_->setState(position, orientation, velocity);
        this->planner_->setPointCloud(point_cloud);
        this->planner_->processPointCloud_();

        double avg_time = 0.0;
        double max_time = 0.0;
        double min_time = 1e3;
        double passed = 0.0;
        int n_loops = 100;
        std::chrono::duration<double> time_passed;
        std::cout << "Running " << n_loops << " loops\n";
        for (int l = 0; l < n_loops; l++) {
            this->planner_->cloud_updated_ = true;

            std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();
            this->planner_->processPointCloud_();
            time_passed = std::chrono::system_clock::now() - time_start;

            passed = time_passed.count();
            avg_time += passed;
            max_time = passed > max_time ? passed : max_time;
            min_time = passed < min_time ? passed : min_time;
        }
        avg_time = avg_time / n_loops;
        std::cout << "Processing time: average: " << avg_time << "; max: " << max_time << "; min: " << min_time << " [s]\n";

    }

    void testTimingRoutePlanning() {
        std::cout << "[TEST] Timing test for route plannig\n";
        this->planner_->reset();

        Eigen::Vector3f position = Eigen::Vector3f::Zero();
        Eigen::Vector4f orientation = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
        Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

        Eigen::Vector3f target = Eigen::Vector3f(1.0f, 1.0f, 0.0f);

        PointCloud<PointXYZ> point_cloud = this->generatePointCloud();

        this->planner_->setState(position, orientation, velocity);
        this->planner_->setTarget(target);
        this->planner_->setPointCloud(point_cloud);
        this->planner_->processPointCloud_();

        double avg_time = 0.0;
        double max_time = 0.0;
        double min_time = 1e3;
        double passed = 0.0;
        int n_loops = 100;
        std::chrono::duration<double> time_passed;
        std::cout << "Running " << n_loops << " loops\n";
        for (int l = 0; l < n_loops; l++) {
            this->planner_->cloud_updated_ = true;

            std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();
            this->planner_->planNext_();
            time_passed = std::chrono::system_clock::now() - time_start;

            passed = time_passed.count();
            avg_time += passed;
            max_time = passed > max_time ? passed : max_time;
            min_time = passed < min_time ? passed : min_time;
        }
        avg_time = avg_time / n_loops;
        std::cout << "Processing time: average: " << avg_time << "; max: " << max_time << "; min: " << min_time << " [s]\n";
    }

protected:

    PointCloud<PointXYZ> generatePointCloud() {

        int c_width = 1280;
        int c_height = 720;
        double step_w = 10.0 / (c_width / 2);
        double step_h = 5.0 / (c_height / 2);
        double step_d = 1.0 / (c_width / 2);
        Eigen::Vector3f np0 = Eigen::Vector3f(5.0, -10.0, -5.0);
        Eigen::Vector3f np = np0;
        Eigen::Vector3f sp = Eigen::Vector3f(step_d, step_w, 0.0);

        PointCloud<PointXYZ> point_cloud = PointCloud<PointXYZ>();
        point_cloud.clear();

        for (int i = 0; i < c_height; i++) {
            np = np0;
            for (int j = 0; j < c_width; j++) {
                point_cloud.push_back(PointXYZ(np.x(), np.y(), np.z()));
                np += sp;
            }
            np.z() += step_h;
        }

        return point_cloud;
    }

    std::unique_ptr<NAVIGATION_CORE::LocalPlanner> planner_;
};


} // namespace NAVIGATION_CORE


int main() {

    int alpha = 6; // [deg]

    NAVIGATION_CORE::PolarHistogramTestClass polar_histogram_test_class = NAVIGATION_CORE::PolarHistogramTestClass();
    polar_histogram_test_class.initHistogram(alpha);
    polar_histogram_test_class.testUpdateCells();

    NAVIGATION_CORE::LocalPlannerConfig planner_config = {
        .enable_cuda = true,
        .goal_dev_margin = 0.1,
        .goal_min_dist = 1.0,
        .planning_step = 1.0
    };

    NAVIGATION_CORE::PlannerTestClass planner_test_class = NAVIGATION_CORE::PlannerTestClass();
    planner_test_class.initPlanner(planner_config);
    planner_test_class.testAddNewGoal();
    planner_test_class.testAddNewGoalIgnore();
    planner_test_class.testUpdateState();
    planner_test_class.testUpdateStateNonZero();
    planner_test_class.testAddNewPointCloud();
    planner_test_class.testProcessPointCloud();
    planner_test_class.testEmptyRun();
    planner_test_class.testTimingAddingPointCloud();
    planner_test_class.testTimingProcessPointCloud();
    planner_test_class.testTimingRoutePlanning();

    return 0;
}
