#pragma once

#include <cmath>
#include <float.h>
#include <vector>
#include <stdexcept>

#include "Eigen/Dense"


namespace NAVIGATION_CORE {

struct HistogramCell {
    HistogramCell() {}
    HistogramCell(float _d, float _a) : distance(_d), age(_a) {}
    float distance = 0.0f;
    float age = 0.0f;
};

class PolarHistogram {

public:

    PolarHistogram();
    PolarHistogram(int alpha);
    ~PolarHistogram() = default;

    inline float getDistance(int y, int x) const {
        return this->distance_(y, x);
    }

    inline float getAge(int y, int x) const {
        return this->age_(y, x);
    }

    inline HistogramCell getCell(int y, int x) const {
        return HistogramCell(this->distance_(y, x), this->age_(y, x));
    }

    inline void setDistance(int y, int x, float value) {
        this->distance_(y, x) = value;
    }

    inline void setAge(int y, int x, float value) {
        this->age_(y, x) = value;
    }

    inline void setCell(int y, int x, float distance, float age) {
        this->distance_(y, x) = distance;
        this->age_(y, x) = age;
    }

    inline void addToDistance(int y, int x, float value) {
        this->distance_(y, x) += value;
    }

    inline void addToAge(int y, int x, float value) {
        this->age_(y, x) += value;
    }

    inline void addToCell(int y, int x, float distance, float age) {
        this->distance_(y, x) += distance;
        this->age_(y, x) += age;
    }

    inline void fillDistance(float value) {
        this->distance_.fill(value);
    }

    inline void fillAge(float value) {
        this->age_.fill(value);
    }

    inline void fillCells(float distance, float age) {
        this->distance_.fill(distance);
        this->age_.fill(age);
    }

    void upsample();
    void downsample();
    void clear();
    bool isEmpty() const;

    int getAlpha() const {return this->alpha_;}
    int getAzimRes() const {return this->azim_dim_;}
    int getElevRes() const {return this->elev_dim_;}

    void wrapIndex(int& y, int& x) const {
        y = y % this->elev_dim_;
        if (y < 0) y += this->elev_dim_;
        x = x % this->azim_dim_;
        if (x < 0) x += this->azim_dim_;
    }

private:

    int alpha_    = 0.0f;
    int azim_dim_ = 0.0f;
    int elev_dim_ = 0.0f;
    
    Eigen::MatrixXf distance_ = {};
    Eigen::MatrixXf age_ = {};

    // inline void wrapIndex(int& y, int& x) const {
    //   y = y % this->elev_dim_;
    //   if (y < 0) y += this->elev_dim_;
    //   x = x % this->azim_dim_;
    //   if (x < 0) x += this->azim_dim_;
    // }

};

} // namespace NAVIGATION_CORE
