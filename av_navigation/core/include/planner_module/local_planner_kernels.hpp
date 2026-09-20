#pragma once


namespace NAVIGATION_CORE_KERNELS {


#define GRID_SIZE 1
#define BLOCK_SIZE 128


constexpr float rad2deg = 180.0f / 3.1415926535897932f;


struct KernelsConfig {
    unsigned alpha = 0;
    unsigned elev_res = 0;
    unsigned azim_res = 0;
    unsigned flat_size = 0;

    float min_distance = 0.0f;
    float max_distance = 0.0f;
    float max_age = 0.0f;

    float fov_h = 0.0f;
    float fov_v = 0.0f;
};


class LocalPlannerKernels {

public:
    LocalPlannerKernels(KernelsConfig config);
    ~LocalPlannerKernels();

    void processIncomingPointCloud(const float* incoming_cloud, unsigned cloud_size);
    float* getHistogramData() const {return this->h_distance_histogram_;}
    float* getAgeData() const {return this->h_age_histogram_;}
    int* getCounterData() const {return this->h_counter_;}

protected:

    void initializeKernels();
    void freeMemory();

    KernelsConfig config_ = {};

    bool initialized_ = false;

    float* h_distance_histogram_ = NULL;
    float* h_age_histogram_ = NULL;
    int* h_counter_ = NULL;

    float* d_confg_f_ = NULL;
    int* d_confg_i_ = NULL;

    float* d_cloud_cache_ = NULL;

    float* d_distance_histogram_ = NULL;
    float* d_age_histogram_ = NULL;
    int* d_counter_ = NULL;
};


} // namespace NAVIGATION_CORE_KERNELS
