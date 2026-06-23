#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

#include "flight_controller.hpp"
#include "state_estimator.hpp"

#include <iostream>
#include <set>
#include <vector>
#include <string>


namespace NAVIGATION_CORE {

class FlightControllerTestClass : public FlightController {

public:
  FlightControllerTestClass() {}
  FlightControllerTestClass(FlightControllerConfig config) : FlightController(config) {}
  ~FlightControllerTestClass() = default;

  void runTest() {

    std::cout << "[INFO] Running 'drone_fligth_controller_test': unit tests\n\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: standstill\n";

    Eigen::Vector3f goal, position, orientation, velocity;
    std::vector<float> rotor_control;

    goal = Eigen::Vector3f(0.0, 0.0, 0.0);
    position = Eigen::Vector3f(0.0, 0.0, 0.0);
    orientation = Eigen::Vector3f(0.0, 0.0, 0.0);
    velocity = Eigen::Vector3f(0.0, 0.0, 0.0);

    this->resetPID();
    this->setGoal(goal);
    rotor_control = this->calculateControl(position, orientation, velocity);

    if (rotor_control[0] == 0 && 
        rotor_control[1] == 0 && 
        rotor_control[2] == 0 && 
        rotor_control[3] == 0)
      std::cout << "[SUCCESS] Rotor control: OK\n";
    else
      std::cout << "[ERROR] Bad rotor contorls\n";
    std::cout << "Print rotor control:\n";
    for (const auto& ctrl : rotor_control) std::cout << ctrl << std::endl;

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: moving upward\n";

    goal = Eigen::Vector3f(0.0, 0.0, 1.0);
    position = Eigen::Vector3f(0.0, 0.0, 0.0);
    orientation = Eigen::Vector3f(0.0, 0.0, 0.0);

    this->resetPID();
    this->setGoal(goal);
    rotor_control = this->calculateControl(position, orientation, velocity);

    if (rotor_control[0] == rotor_control[1] && 
        rotor_control[0] == rotor_control[2] && 
        rotor_control[0] == rotor_control[3] && 
        rotor_control[0] > 0)
      std::cout << "[SUCCESS] Rotor control: OK\n";
    else
      std::cout << "[ERROR] Bad rotor contorls\n";
    std::cout << "Print rotor control:\n";
    for (const auto& ctrl : rotor_control) std::cout << ctrl << std::endl;

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: pitch change\n";

    goal = Eigen::Vector3f(0.0, 0.0, 0.0);
    position = Eigen::Vector3f(0.0, 0.0, 0.0);
    orientation = Eigen::Vector3f(0.0, -0.1, 0.0);

    this->resetPID();
    this->setGoal(goal);
    rotor_control = this->calculateControl(position, orientation, velocity);

    if (rotor_control[0] == rotor_control[1] && 
        rotor_control[0] < 0 && 
        rotor_control[2] == rotor_control[3] && 
        rotor_control[2] > 0)
      std::cout << "[SUCCESS] Rotor control: OK\n";
    else
      std::cout << "[ERROR] Bad rotor contorls\n";
    std::cout << "Print rotor control:\n";
    for (const auto& ctrl : rotor_control) std::cout << ctrl << std::endl;

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: roll change\n";

    goal = Eigen::Vector3f(0.0, 0.0, 0.0);
    position = Eigen::Vector3f(0.0, 0.0, 0.0);
    orientation = Eigen::Vector3f(-0.1, 0.0, 0.0);

    this->resetPID();
    this->setGoal(goal);
    rotor_control = this->calculateControl(position, orientation, velocity);

    if (rotor_control[0] == rotor_control[2] && 
        rotor_control[0] < 0 && 
        rotor_control[1] == rotor_control[3] && 
        rotor_control[1] > 0)
      std::cout << "[SUCCESS] Rotor control: OK\n";
    else
      std::cout << "[ERROR] Bad rotor contorls\n";
    std::cout << "Print rotor control:\n";
    for (const auto& ctrl : rotor_control) std::cout << ctrl << std::endl;

    std::cout << "[INFO] Test STOP\n\n";
  }
};

class StateEstimatorTestClass : public StateEstimator {

public:
  StateEstimatorTestClass() {}
  StateEstimatorTestClass(StateEstimatorConfig config) : StateEstimator(config) {}
  ~StateEstimatorTestClass() = default;

  void runTest() {

    std::cout << "[INFO] Running 'drone_state_estimator_test': unit tests\n\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: output afte initiallization\n";

    Eigen::VectorXf out = this->getState();

    if (out(0)  == 0 && 
        out(1)  == 0 && 
        out(2)  == 0 && 
        out(3)  == 0 && 
        out(4)  == 0 && 
        out(5)  == 0 && 
        out(6)  == 0 && 
        out(7)  == 0 && 
        out(8)  == 0 && 
        out(9)  == 0 && 
        out(10) == 0 && 
        out(11) == 0 )
      std::cout << "[SUCCESS] Initial state: OK\n";
    else
      std::cout << "[ERROR] Wrong initial state\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: running estimation - sanity check\n";

    Eigen::VectorXf input = Eigen::Vector<float, 4>::Zero();
    Eigen::VectorXf measurement = Eigen::Vector<float, 6>::Zero();

    input(0) = 1000.0;
    input(1) = 1000.0;
    input(2) = 1000.0;
    input(3) = 1000.0;

    measurement(0) = 0.1;
    measurement(1) = 0.0;
    measurement(2) = 0.1;

    float dt = 0.01;

    this->runEstimate(input, measurement, dt);
    out = this->getState();

    if (out(0)  != 0 && 
        out(1)  != 0 && 
        out(2)  != 0 && 
        out(3)  != 0 && 
        out(4)  != 0 && 
        out(5)  != 0)
      std::cout << "[SUCCESS] Sanity run: OK\n";
    else {
      std::cout << "[ERROR] State has not been updated after non zero input run\n";
      for (auto &el : out) {
        std::cout << el << std::endl;
      }
    }

    std::cout << "[INFO] Test STOP\n\n";
  }
};

} // namespace NAVIGATION_CORE


int main() {

  NAVIGATION_CORE::FlightControllerConfig flight_controller_config;
  flight_controller_config.reg_mov_z_Kp = 1.0;
  flight_controller_config.reg_rot_x_Kp = 1.0;
  flight_controller_config.reg_rot_y_Kp = 1.0;
  flight_controller_config.reg_rot_z_Kp = 1.0;
  flight_controller_config.pid_min = -1.0;
  flight_controller_config.pid_max =  1.0;

  NAVIGATION_CORE::FlightControllerTestClass flight_controller_test_class = NAVIGATION_CORE::FlightControllerTestClass(flight_controller_config);
  flight_controller_test_class.runTest();

  NAVIGATION_CORE::StateEstimatorConfig state_estimator_config;

  NAVIGATION_CORE::StateEstimatorTestClass state_estimator_test_class = NAVIGATION_CORE::StateEstimatorTestClass(state_estimator_config);
  state_estimator_test_class.runTest();

  return 0;
}