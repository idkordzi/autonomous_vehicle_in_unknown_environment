#include "local_planner_kernels.hpp"


namespace NAVIGATION_CORE_KERNELS {

__global__ void cudaKernelClearHistogram(
    float* distance_histogram,
    float* age_histogram,
    int* counter_,
    unsigned flat_size,
    float max_age)
{
    int id = blockIdx.x * blockDim.x + threadIdx.x;

    if (id < flat_size) {
        distance_histogram[id] = 0.0f;
        age_histogram[id] = max_age;
        counter_[id] = 0;
    }
}

__global__ void cudaKernelProcessIncomingPointCloud(
    float* point_cloud,
    unsigned cloud_size,
    float* distance_histogram,
    int* counter_histogram,
    unsigned histogram_width,
    unsigned alpha,
    float min_distance,
    float max_distance)
{
    int id = blockIdx.x * blockDim.x + threadIdx.x;
    int pi = id*3;

    if (id < cloud_size) {
        float x = point_cloud[pi], y = point_cloud[pi+1], z = point_cloud[pi+2];
        if (isnan(x) || isnan(y) || isnan(z)) return;
        float distanceSq = x*x + y*y + z*z;
        if (distanceSq < min_distance || distanceSq > max_distance) return;

        float den  = sqrtf(x*x + y*y);
        float p_elev = atan2f(z, den) * rad2deg;
        float p_azim = atan2f(y, x) * rad2deg;
        float p_radi = sqrtf(x*x + y*y + z*z);

        unsigned h_azim = min((unsigned)floorf((p_azim + 180.0f) / alpha), (unsigned)360/alpha-1);
        unsigned h_elev = min((unsigned)floorf((p_elev + 90.0f) / alpha), (unsigned)180/alpha-1);
        unsigned hi = h_elev*histogram_width + h_azim;

        atomicAdd(&distance_histogram[hi], p_radi);
        atomicAdd(&counter_histogram[hi], 1);
    }
}

LocalPlannerKernels::LocalPlannerKernels(KernelsConfig config) : config_(config) {
    this->initializeKernels();
}

LocalPlannerKernels::~LocalPlannerKernels() {
    this->freeMemory();
}

void LocalPlannerKernels::initializeKernels() {
    unsigned histogram_size = this->config_.flat_size;

    this->h_distance_histogram_ = (float*)malloc(histogram_size*sizeof(float));
    this->h_age_histogram_ = (float*)malloc(histogram_size*sizeof(float));
    this->h_counter_ = (int*)malloc(histogram_size*sizeof(int));

    if (this->h_distance_histogram_ == NULL || this->h_age_histogram_ == NULL || this->h_counter_ == NULL) {
        // @TODO
    }

    cudaMalloc((void**)&this->d_distance_histogram_, histogram_size*sizeof(float));
    cudaMalloc((void**)&this->d_age_histogram_, histogram_size*sizeof(float));
    cudaMalloc((void**)&this->d_counter_, histogram_size*sizeof(int));

    this->initialized_ = true;
}

void LocalPlannerKernels::processIncomingPointCloud(const float* incoming_cloud, unsigned cloud_size)
{
    if (!this->initialized_) {
        // @TODO throw error
        return;
    }

    int dimGrid  = 0;
    int dimBlock = 0;

    unsigned histogram_size = this->config_.flat_size;

    const void* h_cloud_ptr = (const void*)incoming_cloud;

    // copy incoming point cloud to device
    if (this->d_cloud_cache_ != NULL) cudaFree(this->d_cloud_cache_);
    cudaMalloc((void**)&this->d_cloud_cache_, cloud_size*3*sizeof(float));
    cudaMemcpy((void*)this->d_cloud_cache_, h_cloud_ptr, cloud_size*3*sizeof(float), cudaMemcpyHostToDevice);
  
    // reset histogram values
    dimGrid  = ceil((float)histogram_size / BLOCK_SIZE);
    dimBlock = BLOCK_SIZE;
    cudaKernelClearHistogram<<<dimGrid, dimBlock>>>(
        this->d_distance_histogram_,
        this->d_age_histogram_,
        this->d_counter_,
        this->config_.flat_size,
        this->config_.max_age);

    // process incoming point cloud
    dimGrid  = ceil((float)cloud_size / BLOCK_SIZE);
    dimBlock = BLOCK_SIZE;
    cudaKernelProcessIncomingPointCloud<<<dimGrid, dimBlock>>>(
        this->d_cloud_cache_,
        cloud_size,
        this->d_distance_histogram_,
        this->d_counter_,
        this->config_.azim_res,
        this->config_.alpha,
        this->config_.min_distance,
        this->config_.max_distance);

    // get output histogram from device
    cudaMemcpy(this->h_distance_histogram_, this->d_distance_histogram_, histogram_size*sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(this->h_counter_, this->d_counter_, histogram_size*sizeof(int), cudaMemcpyDeviceToHost);
}

void LocalPlannerKernels::freeMemory() {
    free(this->h_distance_histogram_);
    free(this->h_age_histogram_);
    free(this->h_counter_);

    if (this->d_cloud_cache_ != NULL) cudaFree(this->d_cloud_cache_);

    cudaFree(this->d_distance_histogram_);
    cudaFree(this->d_age_histogram_);
    cudaFree(this->d_counter_);
}

} // namespace NAVIGATION_CORE_KERNELS
