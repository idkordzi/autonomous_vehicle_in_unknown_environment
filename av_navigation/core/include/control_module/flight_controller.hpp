#pragma once

#include <cmath>
#include <cfloat>
#include <vector>
#include <memory>

#include <Eigen/Dense>

#include "pid_controller.hpp"


namespace NAVIGATION_CORE {

constexpr float machine_error = FLT_EPSILON;
constexpr float PI_F = (float)(M_PI);
constexpr float PI_F2 = (float)(M_PI / 2.0f);

struct FlightControllerConfig {

    float hover_ctrl_fr = 0.0f;
    float hover_ctrl_fl = 0.0f;
    float hover_ctrl_rr = 0.0f;
    float hover_ctrl_rl = 0.0f;

    float reg_mov_z_Kp = 0.0f;
    float reg_mov_z_Ki = 0.0f;
    float reg_mov_z_Kd = 0.0f;

    float reg_rot_x_Kp = 0.0f;
    float reg_rot_x_Ki = 0.0f;
    float reg_rot_x_Kd = 0.0f;

    float reg_rot_y_Kp = 0.0f;
    float reg_rot_y_Ki = 0.0f;
    float reg_rot_y_Kd = 0.0f;

    float reg_rot_z_Kp = 0.0f;
    float reg_rot_z_Ki = 0.0f;
    float reg_rot_z_Kd = 0.0f;

    float pid_min = 0.0f;
    float pid_max = 0.0f;

    float ang_x_lim = (float)(M_PI) / 10.0f; // [rad]
    float ang_y_lim = (float)(M_PI) / 10.0f; // [rad]

    float yaw_off_pos_margin = 0.1; // [m]
};

struct DebugStruct {
    float pos_diff_[3] = {0.0};
    float ang_diff_[3] = {0.0};
    float goal_ang_[2] = {0.0};
    float slope_ = 0.0;
};

class FlightController {

public:

    FlightController();
    FlightController(FlightControllerConfig config);
    ~FlightController() = default;

    void setGoal(Eigen::Vector3f goal_position);

    std::vector<float> calculateControl(Eigen::Vector3f position, Eigen::Vector3f orientation, Eigen::Vector3f velocity);

    void resetPID();

    DebugStruct getDebug() {return this->debug_struct_;}
  
protected:

    void initialize();

    FlightControllerConfig config_ = {};

    Eigen::Vector3f goal_position_ = {};

    std::unique_ptr<PID> reg_mov_z_;
    std::unique_ptr<PID> reg_rot_x_;
    std::unique_ptr<PID> reg_rot_y_;
    std::unique_ptr<PID> reg_rot_z_;

    DebugStruct debug_struct_;
};

}  // namespace NAVIGATION_CORE
