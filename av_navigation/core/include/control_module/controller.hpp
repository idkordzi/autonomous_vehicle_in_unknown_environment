#pragma once

#include <cmath>
#include <cfloat>
#include <vector>
#include <memory>

#include <eigen3/Eigen/Dense>

#include "pid.hpp"

#include <iostream>


namespace NAVIGATION_CORE {


constexpr float machine_error = FLT_EPSILON;
constexpr float F_PI = (float)(M_PI);


inline float wrap(float angle) {
    return (angle <= F_PI ? angle : -(2*F_PI - angle));
}


struct ControllerConfig {

    float reg_forward_Kp = 1.0f;
    float reg_forward_Ki = 0.0f;
    float reg_forward_Kd = 0.0f;
    float reg_forward_max_abs = 1.0f; // [m/s]

    float reg_rotate_Kp = 1.0f;
    float reg_rotate_Ki = 0.0f;
    float reg_rotate_Kd = 0.0f;
    float reg_rotate_max_abs = (float)(F_PI) / 10.0f; // [rad/s]

    float max_pos_err_ang_off = 0.1f; // [m]
    float execution_time = 0.01f; // [s]
};


struct DebugStruct {
    float position_error[3] = {0.0f, 0.0f, 0.0f};
    float distance_error = 0.0f;
    float angular_error = 0.0f;
    float resultant_velocity = 0.0f;
    float yaw_rotation = 0.0f;
    float reg_forward_input = 0.0f;
    float reg_rotate_input = 0.0f;
};


class Controller {

public:

    Controller() = delete;
    Controller(ControllerConfig config);
    ~Controller() = default;

    void setGoal(Eigen::Vector3f goal_position);

    std::vector<float> calculateControl(
        Eigen::Vector3f position,
        Eigen::Vector4f orientation,
        Eigen::Vector3f velocity,
        Eigen::Vector3f rotation
    );

    void reset();

    DebugStruct getDebug() {return this->debug_struct_;}

    friend class ControllerTestClass;
  
protected:

    void initialize_();

    ControllerConfig config_ = {};

    Eigen::Vector3f goal_position_ = {};

    std::unique_ptr<PID> reg_forward_;
    std::unique_ptr<PID> reg_rotate_;

    DebugStruct debug_struct_;
};


} // namespace NAVIGATION_CORE
