#pragma once

#include <vector>
#include "Eigen/Dense"


namespace NAVIGATION_CORE {

struct PointXYZ {
    PointXYZ() : x(0.0f), y(0.0f), z(0.0f) {}
    PointXYZ(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

inline Eigen::Vector3f toEigen(const PointXYZ& p) {return Eigen::Vector3f(p.x, p.y, p.z);}
inline PointXYZ toPointXYZ(const Eigen::Vector3f& p) {return PointXYZ(p.x(), p.y(), p.z());}

struct PointXYZI {
    PointXYZI() : x(0.0f), y(0.0f), z(0.0f), i(0.0f) {}
    PointXYZI(float _x, float _y, float _z, float _i) : x(_x), y(_y), z(_z), i(_i) {}
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float i = 0.0f;
};

inline Eigen::Vector3f toEigen(const PointXYZI& p) {return Eigen::Vector3f(p.x, p.y, p.z);}
inline PointXYZI toPointXYZI(const Eigen::Vector3f& p, float age = 0.0f) {return PointXYZI(p.x(), p.y(), p.z(), age);}

template <typename PointType>
class PointCloud {

public:
    PointCloud() = default;
    ~PointCloud() = default;

    using iterator       = typename std::vector<PointType>::iterator;
    using const_iterator = typename std::vector<PointType>::const_iterator;

    iterator begin() {return this->cloud_.begin();}
    const_iterator begin() const {return this->cloud_.begin();}
    const_iterator cbegin() const {return this->cloud_.cbegin();}

    iterator end() {return this->cloud_.end();}
    const_iterator end() const {return this->cloud_.end();}
    const_iterator cend() const {return this->cloud_.cend();}

    void push_back(PointType point) {this->cloud_.push_back(point);}
    void clear() {this->cloud_.clear();}
    size_t size() const {return this->cloud_.size();}

    PointType& operator[](int index) {return this->cloud_[index];}

    std::vector<PointType> cloud_ = {};
};

} // namespace NAVIGATION_CORE
