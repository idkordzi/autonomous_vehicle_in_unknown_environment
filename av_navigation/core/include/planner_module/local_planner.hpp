#pragma once

#include <chrono>
#include <memory>

#include "eigen3/Eigen/Dense"
#include "opencv2/opencv.hpp"

#include "common.hpp"
#include "point_cloud.hpp"
#include "polar_histogram.hpp"
#include "local_planner_kernels.hpp"


namespace NAVIGATION_CORE {


struct LocalPlannerConfig {

    // general params
    bool enable_cuda = true;
    bool skip_planning = false;
    float execution_time = 0.1f; // [s]

    // camera params
    float sensor_range_min = 0.1f; // [m]
    float sensor_range_max = 8.0f; // [m]
    float camera_fov_h = 86.0f; // [deg]
    float camera_fov_v = 58.0f; // [deg]

    // polar histogram params
    int alpha = 3; // [deg]

    // point cloud params
    float point_max_age = 5.0f; // [s]

    // goal settings
    float goal_dev_margin = 0.1f; // [m]
    float goal_min_dist = 4.0f; // [m]

    // trajectory planning
    unsigned max_candidates_per_it = 3;
    float robot_pos_margin = 0.1f; // [m]
    float planning_step = 1.0f; // [m]

    // flight direction cost params
    float cost_yaw = 0.5f;
    float cost_velocity = 1.5f;
    float cost_obstacle_distance = 5000.0f;
    float obstacle_distance_min = 2.0f;
};


struct CostFunctionOutput {
    CostFunctionOutput() : distance_cost(0.0f), state_cost(0.0f) {}
    CostFunctionOutput(float d, float s) : distance_cost(d), state_cost(s) {}

    float distance_cost = 0.0f;
    float state_cost = 0.0f;
};


struct MoveDirection {
    MoveDirection() : elevation(0.0f), azimuth(0.0f), cost(0.0f) {}
    MoveDirection(float _elev, float _azim, float _cost) : elevation(_elev), azimuth(_azim), cost(_cost) {}

    bool operator<(const MoveDirection& d) const {return this->cost < d.cost;}
    bool operator>(const MoveDirection& d) const {return this->cost > d.cost;}
    
    float elevation = 0.0f;
    float azimuth = 0.0f;
    float cost = 0.0f;
};


class LocalPlanner {

public:

    LocalPlanner() = delete;
    LocalPlanner(LocalPlannerConfig config);
    ~LocalPlanner() = default;

    void setState(
        Eigen::Vector3f position,
        Eigen::Vector4f orientation,
        Eigen::Vector3f velocity
    );

    void setTarget(Eigen::Vector3f target);

    void setPointCloud(PointCloud<PointXYZ>& cloud);

    void run();

    Eigen::Vector3f getNext() const;
    
    cv::Mat getHistogramImage() const;

    cv::Mat getCostImage() const;

    void reset();

    friend class PlannerTestClass;

protected:

    void initialize_();

    void processPointCloud_();

    CostFunctionOutput costFunction_(
        const PolarPoint& candidate,
        const Eigen::Vector3f& position,
        const Eigen::Vector3f& velocity,
        float obstacle_distance
    ) const;

    void getCostMatrix_(
        const PolarHistogram& histogram,
        const Eigen::Vector3f& position,
        const Eigen::Vector3f& velocity,
        Eigen::MatrixXf& cost_matrix,
        cv::Mat& cost_image
    ) const;

    void getBestMoveDirections_(
        const Eigen::MatrixXf& cost_matrix,
        std::vector<MoveDirection>& direction_list
    ) const;

    void planNext_();

    void generateHistogramImage_(
        const PolarHistogram& histogram,
        cv::Mat& image_data
    ) const;

    void generateCostImage_(
        const Eigen::MatrixXf& cost_matrix,
        const Eigen::MatrixXf& distance_matrix,
        cv::Mat& image_data
    ) const;
    
    LocalPlannerConfig config_ = {};
    std::unique_ptr<NAVIGATION_CORE_KERNELS::LocalPlannerKernels> kernels_ = {};

    bool state_updated_ = false;
    bool goal_updated_ = false;
    bool cloud_updated_ = false;

    Eigen::Vector3f goal_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f goal_pos_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f next_ = Eigen::Vector3f::Zero();

    Eigen::Vector3f position_ = Eigen::Vector3f::Zero();
    Eigen::Vector4f orientation_ = Eigen::Vector4f::Zero();
    Eigen::Vector3f velocity_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f prev_position_ = Eigen::Vector3f::Zero();

    FOV fov_ = {};

    PointCloud<PointXYZ> cloud_cache_ = {};
    std::chrono::system_clock::time_point last_processing_time_;

    PolarHistogram histogram_;

    cv::Mat image_histogram_ = {};
    cv::Mat image_cost_ = {};
};


} // namespace NAVIGATION_CORE
