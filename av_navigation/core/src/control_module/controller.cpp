#include "controller.hpp"


namespace NAVIGATION_CORE {


Controller::Controller(ControllerConfig config) : config_(config) {
    this->initialize_();
}


void Controller::initialize_() {

    PIDConfig reg_forward_config = {};
    reg_forward_config.Kp = this->config_.reg_forward_Kp;
    reg_forward_config.Ki = this->config_.reg_forward_Ki;
    reg_forward_config.Kd = this->config_.reg_forward_Kd;
    reg_forward_config.out_min = -this->config_.reg_forward_max_abs;
    reg_forward_config.out_max = this->config_.reg_forward_max_abs;
    this->reg_forward_ = std::make_unique<PID>(reg_forward_config);

    PIDConfig reg_rotate_config = {};
    reg_rotate_config.Kp = this->config_.reg_rotate_Kp;
    reg_rotate_config.Ki = this->config_.reg_rotate_Ki;
    reg_rotate_config.Kd = this->config_.reg_rotate_Kd;
    reg_rotate_config.out_min = -this->config_.reg_rotate_max_abs;
    reg_rotate_config.out_max = this->config_.reg_rotate_max_abs;
    this->reg_rotate_ = std::make_unique<PID>(reg_rotate_config);

    this->goal_position_ = Eigen::Vector3f::Zero();
}


void Controller::setGoal(Eigen::Vector3f goal_position) {
    this->goal_position_ = goal_position;
}


std::vector<float> Controller::calculateControl(
    Eigen::Vector3f position,
    Eigen::Vector4f orientation,
    Eigen::Vector3f velocity,
    Eigen::Vector3f rotation
) {

    std::vector<float> control_vector = {0.0, 0.0};  // (1) linear velocity forward (2) angular velocity horizontal

    // calculate error
    Eigen::Vector3f position_error = this->goal_position_ - position;
    float distance_error = position_error.norm();

    float target_angle = std::atan2(position_error.y(), position_error.x());
    Eigen::Vector3f euler_orientation = Eigen::Quaternionf(orientation).normalized().toRotationMatrix().eulerAngles(0, 1, 2);
    float angular_error = target_angle - euler_orientation[2];

    // calculate velocity
    float resultant_velocity = velocity.norm();
    float yaw_rotation = rotation.z();

    // calculate reg input
    float reg_forward_input = -(distance_error - resultant_velocity);
    float reg_rotate_input = -(angular_error - yaw_rotation);

    // safeguard for rotation oscilation
    if (distance_error < this->config_.max_pos_err_ang_off)
        reg_rotate_input = 0.0f;

    // calculate reg output
    float reg_forward_output = this->reg_forward_->calculate(0.0f, reg_forward_input);
    float reg_rotate_output = this->reg_rotate_->calculate(0.0f, reg_rotate_input);

    control_vector[0] = reg_forward_output;
    control_vector[1] = reg_rotate_output;

    // fill debug struct 
    this->debug_struct_.position_error[0] = position_error.x();
    this->debug_struct_.position_error[1] = position_error.y();
    this->debug_struct_.position_error[2] = position_error.z();
    this->debug_struct_.distance_error = distance_error;
    this->debug_struct_.angular_error = angular_error;
    this->debug_struct_.resultant_velocity = resultant_velocity;
    this->debug_struct_.yaw_rotation = yaw_rotation;
    this->debug_struct_.reg_forward_input = reg_forward_input;
    this->debug_struct_.reg_rotate_input = reg_rotate_input;

    return control_vector;
}


void Controller::reset() {
    this->reg_forward_->reset();
    this->reg_rotate_->reset();
    this->goal_position_ = Eigen::Vector3f::Zero();
}


} // namespace NAVIGATION_CORE
