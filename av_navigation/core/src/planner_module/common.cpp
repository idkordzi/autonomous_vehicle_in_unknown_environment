#include "common.hpp"


namespace NAVIGATION_CORE {

bool pointInsideFOV(const FOV& fov, const PolarPoint& polar) {
    return polar.azim <= fov.yaw_deg + fov.h_fov_deg / 2.0f &&
           polar.azim >= fov.yaw_deg - fov.h_fov_deg / 2.0f &&
           polar.elev <= fov.pitch_deg + fov.v_fov_deg / 2.0f &&
           polar.elev >= fov.pitch_deg - fov.v_fov_deg / 2.0f;
}

bool pointInsideFOV(const FOV& fov, const Eigen::Vector3f& point) {
    float den = point.topRows<2>().norm();
    float elev = std::atan2(point.z(), den) * RAD_TO_DEG;
    float azim = std::atan2(point.y(), point.x()) * RAD_TO_DEG;
    return azim <= fov.yaw_deg + fov.h_fov_deg / 2.0f &&
           azim >= fov.yaw_deg - fov.h_fov_deg / 2.0f &&
           elev <= fov.pitch_deg + fov.v_fov_deg / 2.0f &&
           elev >= fov.pitch_deg - fov.v_fov_deg / 2.0f;
}

bool pointInsideFOV(const FOV& fov, const Eigen::Vector3f& point, const Eigen::Vector3f& origin) {
    float den = (point.topRows<2>() - origin.topRows<2>()).norm();
    float elev = std::atan2(point.z() - origin.z(), den) * RAD_TO_DEG;
    float azim = std::atan2(point.y() - origin.y(), point.x() - origin.x()) * RAD_TO_DEG;
    return azim <= fov.yaw_deg + fov.h_fov_deg / 2.0f &&
           azim >= fov.yaw_deg - fov.h_fov_deg / 2.0f &&
           elev <= fov.pitch_deg + fov.v_fov_deg / 2.0f &&
           elev >= fov.pitch_deg - fov.v_fov_deg / 2.0f;
}

PolarPoint convertCartesianToPolar(const Eigen::Vector3f& point) {
    PolarPoint polar(0.0f, 0.0f, 0.0f);
    float den = point.topRows<2>().norm();
    polar.elev = std::atan2(point.z(), den) * RAD_TO_DEG;       // ( -90,  +90)
    polar.azim = std::atan2(point.y(), point.x()) * RAD_TO_DEG; // (-180, +180)
    polar.radi = point.norm();
    return polar;
}

PolarPoint convertCartesianToPolar(const Eigen::Vector3f& point, const Eigen::Vector3f& origin) {
    PolarPoint polar(0.0f, 0.0f, 0.0f);
    float den = (point.topRows<2>() - origin.topRows<2>()).norm();
    polar.elev = std::atan2(point.z() - origin.z(), den) * RAD_TO_DEG;                     // ( -90,  +90)
    polar.azim = std::atan2(point.y() - origin.y(), point.x() - origin.x()) * RAD_TO_DEG;  // (-180, +180)
    polar.radi = sqrt(sqr(point.x() - origin.x()) + sqr(point.y() - origin.y()) + sqr(point.z() - origin.z()));
    return polar;
}

Eigen::Vector3f convertPolarToCartesian(const PolarPoint& polar) {
    Eigen::Vector3f point;
    point.x() = polar.radi * std::cos(polar.elev * DEG_TO_RAD) * std::cos(polar.azim * DEG_TO_RAD);
    point.y() = polar.radi * std::cos(polar.elev * DEG_TO_RAD) * std::sin(polar.azim * DEG_TO_RAD);
    point.z() = polar.radi * std::sin(polar.elev * DEG_TO_RAD);
    return point;
}
    
Eigen::Vector3f convertPolarToCartesian(const PolarPoint& polar, const Eigen::Vector3f& origin) {
    Eigen::Vector3f point;
    point.x() = origin.x() + polar.radi * std::cos(polar.elev * DEG_TO_RAD) * std::cos(polar.azim * DEG_TO_RAD);
    point.y() = origin.y() + polar.radi * std::cos(polar.elev * DEG_TO_RAD) * std::sin(polar.azim * DEG_TO_RAD);
    point.z() = origin.z() + polar.radi * std::sin(polar.elev * DEG_TO_RAD);
    return point;
}
    
// void wrapPolar(PolarPoint& polar) {
//   polar.elev = wrapRange(polar.elev, 180.0f);
//   polar.azim = wrapRange(polar.azim, 360.0f);

//   if (polar.azim > 180.0f)
//     polar.azim = 360.0f - polar.azim;
//   else if (polar.azim < -180.0f)
//     polar.azim = -(360.0f + polar.azim);

//   bool wrapped = false;
//   if (polar.elev > 90.0f) {
//     polar.elev = 180.0f - polar.elev;
//     wrapped = true;
//   } else if (polar.elev < -90.0f) {
//     polar.elev = -(180.0f + polar.elev);
//     wrapped = true;
//   }
  
//   if (wrapped) {
//     if (polar.azim < 0.0f) polar.azim += 180.0f;
//     else polar.azim -= 180.0f;
//   }
// }
    
// Eigen::Vector2i convertPolarToHistogramIndex(const PolarPoint& polar, int alpha) {
//   Eigen::Vector2i idx(0, 0);
//   PolarPoint wrapped = polar;
//   wrapPolar(wrapped);
  
//   idx.y() = (int)(std::floor((wrapped.elev +  90.0f) / alpha));
//   idx.x() = (int)(std::floor((wrapped.azim + 180.0f) / alpha));
  
//   if (idx.y() >= 180 / alpha) idx.y() = 180 / alpha - 1;
//   if (idx.y() < 0) idx.y() = 0;
//   if (idx.x() >= 360 / alpha) idx.x() = 360 / alpha - 1;
//   if (idx.x() < 0) idx.x() = 0;
  
//   return idx;
// }

float angleDifference(float a, float b) {
    float angle = std::fmod(a - b, 360.0f);
    return angle >= 0.0f ? ( (angle < 180.0f) ? angle : angle - 360.0f ) : ( (angle >= -180.0f) ? angle : angle + 360.f );
}

}  // namespace NAVIGATION_CORE
