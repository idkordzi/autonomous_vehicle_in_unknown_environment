#pragma once

#include <cmath>
#include "eigen3/Eigen/Dense"


namespace NAVIGATION_CORE {

// constants

#define PI_F 3.1415926535897932f
#define PI_F2 1.5707963267948966f

constexpr float DEG_TO_RAD = PI_F / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI_F;


// structures

struct PolarPoint {
    PolarPoint(float e_, float z_, float r_) : elev(e_), azim(z_), radi(r_) {};
    PolarPoint() : elev(0.0f), azim(0.0f), radi(0.0f) {};
    float elev = 0.0f;
    float azim = 0.0f;
    float radi = 0.0f;
};


struct FOV {
    FOV() : yaw_deg(0.0f), pitch_deg(0.0f), h_fov_deg(0.0f), v_fov_deg(0.0f) {};
    FOV(float y_, float p_, float h_, float v_) : yaw_deg(y_), pitch_deg(p_), h_fov_deg(h_), v_fov_deg(v_) {};
    float yaw_deg   = 0.0f;
    float pitch_deg = 0.0f;
    float h_fov_deg = 0.0f;
    float v_fov_deg = 0.0f;
};


// functions

inline float clip(float val, float lo_lim = -INFINITY, float hi_lim = INFINITY) {
    return std::min(std::max(val, lo_lim), hi_lim);
}


inline float sqr(float x) {return x*x;}


inline float wrapRange(float angle, float lim) {return (angle - lim * std::floor(angle / lim + 0.5f));}
inline float wrapRangeRad(float angle) {return wrapRange(angle, 2.0f*PI_F);}
inline float wrapRangeDeg(float angle) {return wrapRange(angle, 360.0f);}


bool pointInsideFOV(const FOV& fov, const PolarPoint& polar);
bool pointInsideFOV(const FOV& fov, const Eigen::Vector3f& point);
bool pointInsideFOV(const FOV& fov, const Eigen::Vector3f& point, const Eigen::Vector3f& origin);


PolarPoint convertCartesianToPolar(const Eigen::Vector3f& point);
PolarPoint convertCartesianToPolar(const Eigen::Vector3f& point, const Eigen::Vector3f& origin);


Eigen::Vector3f convertPolarToCartesian(const PolarPoint& polar);
Eigen::Vector3f convertPolarToCartesian(const PolarPoint& polar, const Eigen::Vector3f& origin);


inline void wrapPolar(PolarPoint& polar) {
    if (polar.azim > 180.0f) polar.azim = -360.0f + polar.azim;
    else if (polar.azim < -180.0f) polar.azim = 360.0f + polar.azim;

    if (polar.elev > 90.0f) {
        polar.elev = 180.0f - polar.elev;
        if (polar.azim < 0.0f) polar.azim += 180.0f;
        else polar.azim -= 180.0f;
    } else if (polar.elev < -90.0f) {
        polar.elev = -180.0f - polar.elev;
        if (polar.azim < 0.0f) polar.azim += 180.0f;
        else polar.azim -= 180.0f;
    }
}


inline Eigen::Vector2i convertAngleToHistogramIndex(float elev, float azim, int alpha) {
    return Eigen::Vector2i(std::min(static_cast<int>(std::floor((azim + 180.0f) / alpha)), 360/alpha-1),
                           std::min(static_cast<int>(std::floor((elev +  90.0f) / alpha)), 180/alpha-1));
}


inline Eigen::Vector2i convertPolarToHistogramIndex(const PolarPoint& polar, int alpha) {
    return Eigen::Vector2i(std::min(static_cast<int>(std::floor((polar.azim + 180.0f) / alpha)), 360/alpha-1),
                           std::min(static_cast<int>(std::floor((polar.elev +  90.0f) / alpha)), 180/alpha-1));
}


inline PolarPoint convertHistogramIndexToPolar(int y, int x, int alpha, float radius) {
    return PolarPoint(static_cast<float>(y * alpha + alpha / 2 - 90), static_cast<float>(x * alpha + alpha / 2 - 180), radius);
}


float angleDifference(float a, float b);


} // namespace NAVIGATION_CORE
