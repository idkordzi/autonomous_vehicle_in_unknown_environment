#include "state_estimator.hpp"


namespace NAVIGATION_CORE {

StateEstimator::StateEstimator() {
    this->initialize();
}

StateEstimator::StateEstimator(StateEstimatorConfig config) : config_(config) {
    this->initialize();
}

void StateEstimator::initialize() {
    this->x_  = Eigen::Vector<float,6>::Zero();
    this->xe_ = Eigen::Vector<float,12>::Zero();
    this->Pe_ = Eigen::Matrix<float,12,12>::Zero();

    this->f_  = Eigen::Vector<float,12>::Zero();
    this->h_  = Eigen::Vector<float,12>::Zero();
    this->F_  = Eigen::Matrix<float,12,12>::Zero();
    this->H_  = Eigen::Matrix<float,12,12>::Zero();

    this->Q_  = Eigen::Matrix<float,12,12>::Zero();
    this->R_  = Eigen::Matrix<float,12,12>::Zero();

    this->Q_.diagonal().array() += 1;
    this->R_.diagonal().array() += 1;
}

void StateEstimator::updateModelFunction(Eigen::VectorXf input) {

    float r_fl = input(0), r_fr = input(1), r_rl = input(2), r_rr = input(3);

    this->f_(0)  =  this->xe_(3);
    this->f_(1)  =  this->xe_(4);
    this->f_(2)  =  this->xe_(5);
    this->f_(3)  =  this->config_.g_ * std::sin(this->x_(4))                         + this->xe_(8) * this->xe_(1) - this->xe_(7) * this->xe_(2);
    this->f_(4)  = -this->config_.g_ * std::cos(this->x_(4)) * std::sin(this->x_(3)) + this->xe_(6) * this->xe_(2) - this->xe_(8) * this->xe_(0);
    this->f_(5)  = -this->config_.g_ * std::cos(this->x_(4)) * std::cos(this->x_(3)) + this->xe_(7) * this->xe_(0) - this->xe_(6) * this->xe_(1);
    this->f_(6)  =  this->xe_(9);
    this->f_(7)  =  this->xe_(10);
    this->f_(8)  =  this->xe_(11);
    this->f_(9)  =  this->config_.b1_ * this->xe_(7) * this->xe_(8) + this->config_.c1_ * this->xe_(7) * (-r_fl + r_fr + r_rl - r_rr);
    this->f_(10) =  this->config_.b2_ * this->xe_(8) * this->xe_(6) + this->config_.c2_ * this->xe_(6) * (-r_fl + r_fr + r_rl - r_rr);
    this->f_(11) =  this->config_.b3_ * this->xe_(6) * this->xe_(7);

    this->f_(6)  += this->config_.a1_ * this->config_.Kf * ( r_fl + r_fr + r_rl + r_rr);
    this->f_(9)  += this->config_.a2_ * this->config_.Kf * ( r_fl - r_fr + r_rl - r_rr);
    this->f_(10) += this->config_.a3_ * this->config_.Kf * (-r_fl - r_fr + r_rl + r_rr);
    this->f_(11) += this->config_.a4_ * this->config_.Km * (-r_fl + r_fr + r_rl - r_rr);
}

void StateEstimator::updateModelJacobian(Eigen::VectorXf input, float t) {

    float r_fl = input(0), r_fr = input(1), r_rl = input(2), r_rr = input(3);

    this->F_(0,3) = t;
    this->F_(1,4) = t;
    this->F_(2,5) = t;

    this->F_(3,1) =  this->xe_(8);
    this->F_(3,2) = -this->xe_(7);
    this->F_(3,7) = -this->xe_(2);
    this->F_(3,8) =  this->xe_(1);

    this->F_(4,0) = -this->xe_(8);
    this->F_(4,2) =  this->xe_(6);
    this->F_(4,6) =  this->xe_(2);
    this->F_(4,8) = -this->xe_(0);

    this->F_(5,0) =  this->xe_(7);
    this->F_(5,1) = -this->xe_(6);
    this->F_(5,6) = -this->xe_(1);
    this->F_(5,7) =  this->xe_(0);

    this->F_(6,9)  = t;
    this->F_(7,10) = t;
    this->F_(8,11) = t;

    this->F_(9,7) = this->config_.b1_ * this->xe_(8) + this->config_.c1_ * (-r_fl + r_fr + r_rl - r_rr);
    this->F_(9,8) = this->config_.b1_ * this->xe_(7);

    this->F_(10,6) = this->config_.b2_ * this->xe_(8) + this->config_.c2_ * (-r_fl + r_fr + r_rl - r_rr);
    this->F_(10,8) = this->config_.b2_ * this->xe_(6);

    this->F_(11,6) = this->config_.b3_ * this->xe_(7);
    this->F_(11,7) = this->config_.b3_ * this->xe_(6);
}

void StateEstimator::updateMeasurementFunction(Eigen::VectorXf estims) {
    this->h_(3) = estims(3);
    this->h_(4) = estims(4);
    this->h_(5) = estims(5);
    this->h_(6) = estims(6);
    this->h_(7) = estims(7);
    this->h_(8) = estims(8);
}

void StateEstimator::updateMeasurementJacobian() {
    this->H_(3,3) = 1;
    this->H_(4,4) = 1;
    this->H_(5,5) = 1;
    this->H_(6,6) = 1;
    this->H_(7,7) = 1;
    this->H_(8,8) = 1;
}

void StateEstimator::runEstimate(Eigen::VectorXf input, Eigen::VectorXf measurement) {
    std::chrono::duration<double> time_passed = std::chrono::system_clock::now() - this->ts_;
    this->runEstimate(input, measurement, time_passed.count());
    this->ts_ = std::chrono::system_clock::now();
}

void StateEstimator::runEstimate(Eigen::VectorXf input, Eigen::VectorXf measurement, float dt) {

    // (*) drone state update
    this->x_(0) = this->xe_(0) * dt;
    this->x_(1) = this->xe_(1) * dt;
    this->x_(2) = this->xe_(2) * dt;
    this->x_(3) = this->xe_(6) * dt;
    this->x_(4) = this->xe_(7) * dt;
    this->x_(5) = this->xe_(8) * dt;

    // prediction step
    this->updateModelFunction(input);
    this->updateModelJacobian(input, dt);

    Eigen::VectorXf xp = this->f_;
    Eigen::MatrixXf Pp = this->F_ * this->Pe_ * this->F_.transpose() + this->Q_;

    // update step
    Eigen::VectorXf m = Eigen::Vector<float, 12>::Zero();
    m(3) = measurement(0);
    m(4) = measurement(1);
    m(5) = measurement(2);
    m(6) = measurement(3);
    m(7) = measurement(4);
    m(8) = measurement(5);
    this->updateMeasurementFunction(xp);
    this->updateMeasurementJacobian();

    Eigen::VectorXf y = m - this->h_;
    Eigen::MatrixXf S = this->H_ * Pp * this->H_.transpose() + this->R_;
    Eigen::MatrixXf K = Pp * this->H_.transpose() * S.inverse();

    // update output
    this->xe_ = xp + K * y;
    this->Pe_ = Pp - K * this->H_ * Pp;
}

Eigen::VectorXf StateEstimator::getState() const {
    Eigen::VectorXf out = Eigen::Vector<float, 12>::Zero();
    out(0)  = this->x_(0);
    out(1)  = this->x_(1);
    out(2)  = this->x_(2);
    out(3)  = this->xe_(0);
    out(4)  = this->xe_(1);
    out(5)  = this->xe_(2);
    out(6)  = this->x_(3);
    out(7)  = this->x_(4);
    out(8)  = this->x_(5);
    out(9)  = this->xe_(6);
    out(10) = this->xe_(7);
    out(11) = this->xe_(8);
    return out;
}

}  // namespace NAVIGATION_CORE
