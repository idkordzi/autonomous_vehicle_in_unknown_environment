#include "flight_controller.hpp"


namespace NAVIGATION_CORE {

FlightController::FlightController() {
    this->initialize();
}

FlightController::FlightController(FlightControllerConfig config) : config_(config) {
    this->initialize();
}

void FlightController::initialize() {

    this->reg_mov_z_ = std::make_unique<PID>(this->config_.reg_mov_z_Kp, this->config_.reg_mov_z_Ki, this->config_.reg_mov_z_Kd, this->config_.pid_min, this->config_.pid_max);
    this->reg_rot_x_ = std::make_unique<PID>(this->config_.reg_rot_x_Kp, this->config_.reg_rot_x_Ki, this->config_.reg_rot_x_Kd, this->config_.pid_min, this->config_.pid_max);
    this->reg_rot_y_ = std::make_unique<PID>(this->config_.reg_rot_y_Kp, this->config_.reg_rot_y_Ki, this->config_.reg_rot_y_Kd, this->config_.pid_min, this->config_.pid_max);
    this->reg_rot_z_ = std::make_unique<PID>(this->config_.reg_rot_z_Kp, this->config_.reg_rot_z_Ki, this->config_.reg_rot_z_Kd, this->config_.pid_min, this->config_.pid_max);  
}

void FlightController::setGoal(Eigen::Vector3f goal_position) {
    this->goal_position_ = goal_position;
}

std::vector<float> FlightController::calculateControl(Eigen::Vector3f position, Eigen::Vector3f orientation, Eigen::Vector3f velocity) {

    // std::vector<float> control_vector = {
    //   this->config_.hover_ctrl_fr_, // front right // ccw
    //   this->config_.hover_ctrl_fl_, // front left  // cw
    //   this->config_.hover_ctrl_rr_, // rear right  // cw
    //   this->config_.hover_ctrl_rl_, // rear left   // ccw
    // };

    std::vector<float> control_vector = {0.0, 0.0, 0.0, 0.0};

    // convert goal point to drone local CS
    float angle = orientation.z();
    Eigen::Vector3f goal_new = this->goal_position_ - position;
    Eigen::Vector3f goal_rotated = Eigen::Vector3f(
        goal_new.x() * std::cos(angle) + goal_new.y() * std::sin(angle),
        goal_new.x() * -std::sin(angle) + goal_new.y() * std::cos(angle),
        goal_new.z()
    );

    Eigen::Vector3f velocity_local = Eigen::Vector3f(
        velocity.x() * std::cos(angle) + velocity.y() * std::sin(angle),
        velocity.x() * -std::sin(angle) + velocity.y() * std::cos(angle),
        velocity.z()
    );
 
    Eigen::Vector3f pos_diff = -goal_rotated;

    // float goal_ang_x = clip(std::atan( pos_diff.y()), -this->config_.ang_x_lim, this->config_.ang_x_lim);
    float goal_ang_x = 0.0; // focus on yaw change instead of linear movement
    
    float goal_ang_y = clip(std::atan(-pos_diff.x()), -this->config_.ang_y_lim, this->config_.ang_y_lim);

    float slope = goal_rotated.y() / (goal_rotated.x() + machine_error);
    float yaw_diff = -std::atan(slope);
    if (pos_diff.x() >= 0) {
        if (pos_diff.y() < 0)      yaw_diff = (float)(-M_PI) + yaw_diff;
        else if (pos_diff.y() > 0) yaw_diff = (float)(M_PI) - std::abs(yaw_diff);
    }

    Eigen::Vector3f ang_diff = Eigen::Vector3f(orientation.x() - goal_ang_x, orientation.y() - goal_ang_y, yaw_diff);

    Eigen::Vector4f input_error = Eigen::Vector4f(
        pos_diff.z(),
        ang_diff.x() - 0.7*velocity_local.y(),
        ang_diff.y() + 0.7*velocity_local.x(),
        ang_diff.z()
    );

    // turn off yaw control when near goal position
    if (goal_rotated.head<2>().norm() < this->config_.yaw_off_pos_margin)
        input_error[3] = 0.0f;

    // allow back off w/o yaw control / no yaw control if yaw diff > PI
    if (std::abs(yaw_diff) > PI_F2)
        input_error[3] = 0.0f;

    // Movement along Z axis (UPSIDE/DOWNSIDE)
    float ctrl_mov_z = this->reg_mov_z_->calculate(0.0f, input_error[0]);
    control_vector[0] += ctrl_mov_z;
    control_vector[1] += ctrl_mov_z;
    control_vector[2] += ctrl_mov_z;
    control_vector[3] += ctrl_mov_z;

    // Rotation around X axis (ROLL) / movement along Y axis (SIDEWARD)
    float ctrl_rot_x = this->reg_rot_x_->calculate(0.0f, input_error[1]);
    control_vector[0] += -ctrl_rot_x;
    control_vector[1] +=  ctrl_rot_x;
    control_vector[2] += -ctrl_rot_x;
    control_vector[3] +=  ctrl_rot_x;

    // Rotation around Y axis (PITCH) / movement along X axis (FORWARD/BACKWARD)
    float ctrl_rot_y = this->reg_rot_y_->calculate(0.0f, input_error[2]);
    control_vector[0] += -ctrl_rot_y;
    control_vector[1] += -ctrl_rot_y;
    control_vector[2] +=  ctrl_rot_y;
    control_vector[3] +=  ctrl_rot_y;

    // Rotation around Z axis (YAW)
    float ctrl_rot_z = this->reg_rot_z_->calculate(0.0f, input_error[3]);
    control_vector[0] += -ctrl_rot_z;
    control_vector[1] +=  ctrl_rot_z;
    control_vector[2] +=  ctrl_rot_z;
    control_vector[3] += -ctrl_rot_z;

    control_vector[0] = control_vector[0] / 4 + this->config_.hover_ctrl_fr;
    control_vector[1] = control_vector[1] / 4 + this->config_.hover_ctrl_fl;
    control_vector[2] = control_vector[2] / 4 + this->config_.hover_ctrl_rr;
    control_vector[3] = control_vector[3] / 4 + this->config_.hover_ctrl_rl;

    debug_struct_.pos_diff_[0] = pos_diff.x();
    debug_struct_.pos_diff_[1] = pos_diff.y();
    debug_struct_.pos_diff_[2] = pos_diff.z();
    debug_struct_.ang_diff_[0] = ang_diff.x();
    debug_struct_.ang_diff_[1] = ang_diff.y();
    debug_struct_.ang_diff_[2] = ang_diff.z();
    debug_struct_.goal_ang_[0] = goal_ang_x;
    debug_struct_.goal_ang_[1] = goal_ang_y;
    debug_struct_.slope_ = slope;

    // [1]

    return control_vector;
}

void FlightController::resetPID() {
    this->reg_mov_z_->reset();
    this->reg_rot_x_->reset();
    this->reg_rot_y_->reset();
    this->reg_rot_z_->reset();
}

}  // namespace NAVIGATION_CORE



// [1]

// mz =  a + b + c + d -> a = mz - b - c - d
// rx = -a + b - c + d -> b = rx + a + c - d
// ry = -a - b + c + d -> c = ry + a + b - d
// rz =  a - b - c + d -> d = rz - a + b + c

// a = mz - b - c - d
// b = rx + ( mz - b - c - d ) + c - d
//   = rx + mz - b - 2d
//   -> b = rx/2 + mz/2 - d
// c = ry + ( mz - ( rx/2 + mz/2 - d ) - c - d ) + ( rx/2 + mz/2 - d ) - d
//   = ry + mz - rx/2 - mz/2 + d - c - d + rx/2 + mz/2 - d - d 
//   = ry + mz - c - 2d
//   -> c = ry/2 + mz/2 - d
// d = rz - ( mz - ( rx/2 + mz/2 - d ) - ( ry/2 + mz/2 - d ) - d ) + ( rx/2 + mz/2 - d ) + ( ry/2 + mz/2 - d )
//   = rz - mz + rx/2 + mz/2 - d + ry/2 + mz/2 - d + d + rx/2 + mz/2 - d + ry/2 + mz/2 - d
//   = rz + mz + rx + ry - 3d
//   -> d = mz/4 + rx/4 + ry/4 + rz/4

// a = mz - b - c - d
// b = rx/2 + mz/2 - d
// c = ry/2 + mz/2 - d
// d = mz/4 + rx/4 + ry/4 + rz/4

// a = mz - mz/4 - rx/4 + ry/4 + rz/4 - mz/4 + rx/4 - ry/4 + rz/4 - mz/4 - rx/4 - ry/4 - rz/4
//   = mz/4 - rx/4 - ry/4 + rz/4
// b = mz/4 + rx/4 - ry/4 - rz/4
// c = mz/4 - rx/4 + ry/4 - rz/4
// d = mz/4 + rx/4 + ry/4 + rz/4

// ctrl_mov_z /= 4;
// ctrl_rot_x /= 4;
// ctrl_rot_y /= 4;
// ctrl_rot_z /= 4;

// control_vector[0] += ctrl_mov_z - ctrl_rot_x - ctrl_rot_y + ctrl_rot_z;
// control_vector[1] += ctrl_mov_z + ctrl_rot_x - ctrl_rot_y - ctrl_rot_z;
// control_vector[2] += ctrl_mov_z - ctrl_rot_x + ctrl_rot_y - ctrl_rot_z;
// control_vector[3] += ctrl_mov_z + ctrl_rot_x + ctrl_rot_y + ctrl_rot_z;
