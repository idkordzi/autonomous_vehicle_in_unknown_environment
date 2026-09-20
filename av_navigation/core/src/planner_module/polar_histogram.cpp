#include "polar_histogram.hpp"


namespace NAVIGATION_CORE {


PolarHistogram::PolarHistogram() {
    this->clear();
}


PolarHistogram::PolarHistogram(int alpha)
: alpha_{alpha}, azim_dim_{360 / alpha}, elev_dim_{180 / alpha}, distance_(elev_dim_, azim_dim_), age_(elev_dim_, azim_dim_)
{
    this->clear();
}


void PolarHistogram::upsample() {
    if (this->alpha_ % 2 != 0) {
        throw std::logic_error("Invalid use of function 'upsample()'. Cell size must be divisiable by 2.");
    }
    this->alpha_ = this->alpha_ / 2;
    this->azim_dim_ = 2 * this->azim_dim_;
    this->elev_dim_ = 2 * this->elev_dim_;
    Eigen::MatrixXf new_histogram(this->elev_dim_, this->azim_dim_);

    for (int i = 0; i < this->elev_dim_; i++) {
        for (int j = 0; j < this->azim_dim_; j++) {
            int i_low_res = floor(i / 2);
            int j_low_res = floor(j / 2);
            new_histogram(i, j) = this->distance_(i_low_res, j_low_res);
        }
    }
    this->distance_ = new_histogram;
}


void PolarHistogram::downsample() {
    if (this->azim_dim_ % 2 != 0 || this->elev_dim_ % 2 != 0) {
        throw std::logic_error("Invalid use of function downsample(). Histogram resulotion for elevation and azimuth must be divisable by 2.");
    }
    this->alpha_ = 2 * this->alpha_;
    this->azim_dim_ = this->azim_dim_ / 2;
    this->elev_dim_ = this->elev_dim_ / 2;
    Eigen::MatrixXf new_histogram(elev_dim_, azim_dim_);

    for (int i = 0; i < this->elev_dim_; i++) {
        for (int j = 0; j < this->azim_dim_; j++) {
            int i_high_res = 2 * i;
            int j_high_res = 2 * j;
            new_histogram(i, j) = this->distance_.block(i_high_res, j_high_res, 2, 2).mean();
        }
    }
    this->distance_ = new_histogram;
}


void PolarHistogram::clear() {
    this->distance_.fill(0.0f);
    this->age_.fill(0.0f);
}


bool PolarHistogram::isEmpty() const {
    bool is_empty = true;
    for (int e = 0; (e < elev_dim_) && (is_empty); e++) {
        for (int z = 0; (z < azim_dim_) && (is_empty); z++) {
            if (this->distance_(e, z) > FLT_MIN) is_empty = false;
        }
    }
    return is_empty;
}


} // namespace NAVIGATION_CORE
