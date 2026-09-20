#include <iostream>
#include <set>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <chrono>

#include "eigen3/Eigen/Dense"

#include "controller.hpp"


namespace NAVIGATION_CORE {


inline bool eqm(float a, float b, float m) {
    return std::abs(a-b) <= m;
}


class ControllerTestClass {

public:

    ControllerTestClass() = default;
    ~ControllerTestClass() = default;

    void initController(ControllerConfig config) {
        std::cout << "[INFO] Initializing CONTROLLER class\n";
        this->controller = std::make_unique<Controller>(config);
    }

    void testStandstill() {
        std::cout << "[TEST] Standstill\n";

        Eigen::Vector3f
            goal = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            position = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            rotation = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector4f orientation = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
        std::vector<float> control_vector;

        this->controller->reset();
        this->controller->setGoal(goal);

        std::this_thread::sleep_for(std::chrono::milliseconds((long)(this->controller->config_.execution_time * 1e3)));
        control_vector = this->controller->calculateControl(position, orientation, velocity, rotation);

        if (control_vector[0] == 0.0f && control_vector[1] == 0.0f)
            std::cout << "[SUCCESS] Control: OK\n";
        else
            std::cout << "[ERROR] Invalid values on control signal\n";

        std::cout << "[INFO] Print rotor control:\n";
        for (const auto& control : control_vector)
            std::cout << control << std::endl;
    }

    void testMoveForward() {
        std::cout << "[TEST] Moving forward\n";

        Eigen::Vector3f
            goal = Eigen::Vector3f(1.0f, 0.0f, 0.0f),
            position = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            rotation = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector4f orientation = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
        std::vector<float> control_vector;

        this->controller->reset();
        this->controller->setGoal(goal);

        std::this_thread::sleep_for(std::chrono::milliseconds((long)(this->controller->config_.execution_time * 1e3)));
        control_vector = this->controller->calculateControl(position, orientation, velocity, rotation);

        if (control_vector[0] > 0.0f && control_vector[1] == 0.0f)
            std::cout << "[SUCCESS] Control: OK\n";
        else
            std::cout << "[ERROR] Invalid values on control signal\n";

        std::cout << "[INFO] Print rotor control:\n";
        for (const auto& control : control_vector)
            std::cout << control << std::endl;
    }

    void testMoveForwardNonZeroVelocity() {
        std::cout << "[TEST] Moving forward with non-zero velocity\n";

        Eigen::Vector3f
            goal = Eigen::Vector3f(1.0f, 0.0f, 0.0f),
            position = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            velocity = Eigen::Vector3f(1.0f, 0.0f, 0.0f),
            rotation = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector4f orientation = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
        std::vector<float> control_vector;

        this->controller->reset();
        this->controller->setGoal(goal);

        std::this_thread::sleep_for(std::chrono::milliseconds((long)(this->controller->config_.execution_time * 1e3)));
        control_vector = this->controller->calculateControl(position, orientation, velocity, rotation);

        if (control_vector[0] == 0.0f && control_vector[1] == 0.0f)
            std::cout << "[SUCCESS] Control: OK\n";
        else
            std::cout << "[ERROR] Invalid values on control signal\n";

        std::cout << "[INFO] Print rotor control:\n";
        for (const auto& control : control_vector)
            std::cout << control << std::endl;
    }

    void testRotateRight() {
        std::cout << "[TEST] Rotate right\n";

        Eigen::Vector3f
            goal = Eigen::Vector3f(1.0f, 1.0f, 0.0f),
            position = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            rotation = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector4f orientation = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
        std::vector<float> control_vector;

        this->controller->reset();
        this->controller->setGoal(goal);

        std::this_thread::sleep_for(std::chrono::milliseconds((long)(this->controller->config_.execution_time * 1e3)));
        control_vector = this->controller->calculateControl(position, orientation, velocity, rotation);

        if (control_vector[0] > 0.0f && control_vector[1] > 0.0f)
            std::cout << "[SUCCESS] Control: OK\n";
        else
            std::cout << "[ERROR] Invalid values on control signal\n";

        std::cout << "[INFO] Print rotor control:\n";
        for (const auto& control : control_vector)
            std::cout << control << std::endl;
    }

    void testNoRotateWhenClose() {
        std::cout << "[TEST] No rotate when close to target\n";

        Eigen::Vector3f
            goal = Eigen::Vector3f(0.05f, 0.05f, 0.0f),
            position = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            rotation = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector4f orientation = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
        std::vector<float> control_vector;

        this->controller->reset();
        this->controller->setGoal(goal);

        std::this_thread::sleep_for(std::chrono::milliseconds((long)(this->controller->config_.execution_time * 1e3)));
        control_vector = this->controller->calculateControl(position, orientation, velocity, rotation);

        if (control_vector[0] > 0.0f && control_vector[1] == 0.0f)
            std::cout << "[SUCCESS] Control: OK\n";
        else
            std::cout << "[ERROR] Invalid values on control signal\n";

        std::cout << "[INFO] Print rotor control:\n";
        for (const auto& control : control_vector)
            std::cout << control << std::endl;
    }

    void testRotateNonZeroYaw() {
        std::cout << "[TEST] No rotate when close to target\n";

        Eigen::Vector3f
            goal = Eigen::Vector3f(1.0f, 1.0f, 0.0f),
            position = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f),
            rotation = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        // Eigen::Vector4f orientation = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
        Eigen::Vector4f orientation = Eigen::Vector4f(0.0f, 0.0f, -0.2588192f, 0.9659258f); // -30.0 deg to left
        // Eigen::Vector4f orientation = Eigen::Vector4f(0.0f, 0.0f, 0.3826834f, 0.9238796f); // 45 deg to right
        // Eigen::Vector4f orientation = Eigen::Vector4f(0.0f, 0.0f, 0.5000011f, 0.8660248f); // 60 deg to right
        std::vector<float> control_vector;

        // Eigen::Vector3f euler = Eigen::Quaternionf(orientation).normalized().toRotationMatrix().eulerAngles(0, 1, 2);
        // std::cout << "Current orientation: " << euler[0] << " " << euler[1] << " " << euler[2] << " [XYZ]" << std::endl;
        // float target_yaw = std::atan2((goal - position).y(), (goal - position).x());
        // std::cout << "Target yaw: " << target_yaw << std::endl;

        this->controller->reset();
        this->controller->setGoal(goal);

        std::this_thread::sleep_for(std::chrono::milliseconds((long)(this->controller->config_.execution_time * 1e3)));
        control_vector = this->controller->calculateControl(position, orientation, velocity, rotation);

        if (control_vector[0] > 0.0f && control_vector[1] > 0.0f)
            std::cout << "[SUCCESS] Control: OK\n";
        else
            std::cout << "[ERROR] Invalid values on control signal\n";

        std::cout << "[INFO] Print rotor control:\n";
        for (const auto& control : control_vector)
            std::cout << control << std::endl;

        // this->print_debug_();
    }

protected:

    void print_debug_() {
        std::cout << "[INFO] Printing debug" << std::endl;
        std::cout << "- position error: "
            << this->controller->debug_struct_.position_error[0] << " "
            << this->controller->debug_struct_.position_error[1] << " "
            << this->controller->debug_struct_.position_error[2]
            << std::endl;
        std::cout << "- distance error: " << this->controller->debug_struct_.distance_error << std::endl;
        std::cout << "- angular error: " << this->controller->debug_struct_.angular_error << std::endl;
        std::cout << "- resultant velocity: " << this->controller->debug_struct_.resultant_velocity << std::endl;
        std::cout << "- yaw rotation: " << this->controller->debug_struct_.yaw_rotation<< std::endl;
        std::cout << "- input reg vel: " << this->controller->debug_struct_.reg_forward_input << std::endl;
        std::cout << "- input reg ang: " << this->controller->debug_struct_.reg_rotate_input << std::endl;
    }

    std::unique_ptr<Controller> controller;

};


} // namespace NAVIGATION_CORE


int main() {

    NAVIGATION_CORE::ControllerTestClass controller_test_class = NAVIGATION_CORE::ControllerTestClass();

    NAVIGATION_CORE::ControllerConfig controller_config = {
        .reg_forward_Kp = 1.0f,
        .reg_forward_Ki = 0.0f,
        .reg_forward_Kd = 0.0f,
        .reg_forward_max_abs = INFINITY,
        .reg_rotate_Kp = 1.0f,
        .reg_rotate_Ki = 0.0f,
        .reg_rotate_Kd = 0.0f,
        .reg_rotate_max_abs = INFINITY,
        .max_pos_err_ang_off = 0.1f,
        .execution_time = 0.1f
    };

    controller_test_class.initController(controller_config);
    controller_test_class.testStandstill();
    controller_test_class.testMoveForward();
    controller_test_class.testMoveForwardNonZeroVelocity();
    controller_test_class.testRotateRight();
    controller_test_class.testNoRotateWhenClose();
    controller_test_class.testRotateNonZeroYaw();

    return 0;
}