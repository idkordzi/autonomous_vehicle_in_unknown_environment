#include "pid_controller.hpp"


namespace NAVIGATION_CORE {

PID::PID(float Kp, float Ki, float Kd, float min_lim, float max_lim) {
    this->config_.Kp = Kp;
    this->config_.Ki = Ki;
    this->config_.Kd = Kd;
    this->config_.out_min = min_lim;
    this->config_.out_max = max_lim;
}

PID::PID(PIDConfig config) : config_(config) {}

float PID::calculate(float sp, float pv) {
    std::chrono::duration<double> time_passed = std::chrono::system_clock::now() - this->ts_;
    float output = this->calculate(sp, pv, time_passed.count());
    this->ts_ = std::chrono::system_clock::now();
    return output;
}

float PID::calculate(float sp, float pv, float dt) {
    dt = clip(dt, this->config_.min_dt, this->config_.max_dt);

    float error = sp - pv;
    float Pout = this->config_.Kp * error;

    this->mem_int_ += error * dt;
    float Iout = this->config_.Ki * this->mem_int_;

    float deriv = (error - this->mem_err_) / dt;
    float Dout = this->config_.Kd * deriv;

    float output = clip(Pout + Iout + Dout, this->config_.out_min, this->config_.out_max);
    this->mem_err_ = error;

    return output;
}

void PID::reset() {
    this->ts_ = std::chrono::system_clock::now();
    this->mem_err_ = 0.0f;
    this->mem_int_ = 0.0f;
}

}  // namespace NAVIGATION_CORE
