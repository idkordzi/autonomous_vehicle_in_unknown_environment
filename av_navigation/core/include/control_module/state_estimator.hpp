#pragma once

#include <chrono>
#include "Eigen/Dense"


namespace NAVIGATION_CORE {

struct StateEstimatorConfig {
    const unsigned state_num_ = 1;
    const unsigned measurement_num_ = 1;

    // model parameters
    const float g_ = 9.81f;

    const float a1_ = 1.0f / 1.5f;
    const float a2_ = (std::sqrt(2) * 0.235f) / (2 * 0.008f);
    const float a3_ = (std::sqrt(2) * 0.235f) / (2 * 0.015f);
    const float a4_ = 1.0f / 0.017f;

    const float b1_ = (0.015f - 0.017f) / 0.008f;
    const float b2_ = (0.017f - 0.008f) / 0.015f;
    const float b3_ = (0.008f - 0.015f) / 0.017f;

    const float c1_ = -0.000167604f / 0.008f;
    const float c2_ =  0.000167604f / 0.015f;

    const float Kf = 0.00001f;
    const float Km = 0.00001f;
};

// for hover velocity, ctrl is 1534.0 which gives 0.5425 PWM which gives ~454.615 RPM
// for hover velocity Kf = g / hover^2 / 4

class StateEstimator {

public:

    StateEstimator();
    StateEstimator(StateEstimatorConfig config);
    ~StateEstimator() = default;

    void runEstimate(Eigen::VectorXf input, Eigen::VectorXf measurement);
    void runEstimate(Eigen::VectorXf input, Eigen::VectorXf measurement, float dt);

    Eigen::VectorXf getState() const;

protected:

    void initialize();

    void updateModelFunction(Eigen::VectorXf input);
    void updateModelJacobian(Eigen::VectorXf input, float t);

    void updateMeasurementFunction(Eigen::VectorXf estims);
    void updateMeasurementJacobian();

    StateEstimatorConfig config_ = {};

    std::chrono::system_clock::time_point ts_ = {};

    Eigen::VectorXf x_ = {}; // drone position and orientation

    Eigen::VectorXf xe_ = {}; // states estimates
    Eigen::MatrixXf Pe_ = {}; // estimations covariance

    Eigen::MatrixXf f_ = {}; // model function
    Eigen::MatrixXf h_ = {}; // measurements function

    Eigen::MatrixXf F_ = {}; // model Jacobian
    Eigen::MatrixXf H_ = {}; // measurements Jacobian

    Eigen::MatrixXf Q_ = {}; // model covariance matrix
    Eigen::MatrixXf R_ = {}; // measurements covariance matrix
};

}  // namespace NAVIGATION_CORE
