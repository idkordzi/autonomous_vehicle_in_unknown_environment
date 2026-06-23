#pragma once

#include <cmath>
#include <chrono>


namespace NAVIGATION_CORE {

inline float clip(float val, float lo_lim = -INFINITY, float hi_lim = INFINITY) {
  return std::min(std::max(val, lo_lim), hi_lim);
}

struct PIDConfig {
  float min_dt = 1e-15; // [s]
  float max_dt = 1.0;   // [s]

  float Kp = 1.0f;
  float Ki = 0.0f;
  float Kd = 0.0f;

  float out_min = -INFINITY;
  float out_max = INFINITY;
};

class PID {

public:

  PID() = delete;
  PID(float Kp, float Ki, float Kd, float min_lim = -INFINITY, float max_lim = INFINITY);
  PID(PIDConfig config);
  ~PID() = default;

  float calculate(float sp, float pv);

  float calculate(float sp, float pv, float dt);

  void reset();

private:

  PIDConfig config_ = {};

  std::chrono::system_clock::time_point ts_ = {};

  float mem_err_ = 0.0f;
  float mem_int_ = 0.0f;
};

} // namespace NAVIGATION_CORE
