#include "local_planner.hpp"


namespace NAVIGATION_CORE {


LocalPlanner::LocalPlanner(LocalPlannerConfig config) : config_(config) {
    this->initialize_();
}


void LocalPlanner::initialize_() {

    this->histogram_ = PolarHistogram(this->config_.alpha);
    this->image_histogram_ = cv::Mat(this->histogram_.getElevRes(), this->histogram_.getAzimRes(), CV_8UC3, cv::Scalar(0,0,0));
    this->image_cost_ = cv::Mat(this->histogram_.getElevRes(), this->histogram_.getAzimRes(), CV_8UC3, cv::Scalar(0,0,0));

    this->reset();

    NAVIGATION_CORE_KERNELS::KernelsConfig k_config;
    k_config.alpha = this->config_.alpha;
    k_config.elev_res = this->histogram_.getElevRes();
    k_config.azim_res = this->histogram_.getAzimRes();
    k_config.flat_size = k_config.elev_res * k_config.azim_res;
    k_config.min_distance = this->config_.sensor_range_min;
    k_config.max_distance = this->config_.sensor_range_max;
    k_config.max_age = this->config_.point_max_age;
    this->kernels_ = std::make_unique<NAVIGATION_CORE_KERNELS::LocalPlannerKernels>(k_config);
}


void LocalPlanner::setState(Eigen::Vector3f position, Eigen::Vector4f orientation, Eigen::Vector3f velocity) {

    this->prev_position_ = this->position_;

    this->position_ = position;
    this->orientation_ = orientation;
    this->velocity_ = velocity;

    Eigen::Vector3f euler = Eigen::Quaternionf(orientation).normalized().toRotationMatrix().eulerAngles(0, 1, 2);
    this->fov_.pitch_deg = euler.y() * RAD_TO_DEG;
    this->fov_.yaw_deg = euler.z() * RAD_TO_DEG;

    this->state_updated_ = true;
}


void LocalPlanner::setTarget(Eigen::Vector3f goal) {

    Eigen::Vector3f new_goal = goal;
    if (this->it_since_last_update_ > 0 || (this->goal_ - new_goal).norm() > this->config_.goal_dev_margin) {

        this->goal_ = new_goal;

        if (this->target_array_.size() >= this->config_.max_past_targets)
            this->target_array_.pop_back();
        this->target_array_.insert(this->target_array_.cbegin(), new_goal);

        PolarPoint facing_goal = convertCartesianToPolar(this->goal_, this->position_);
        PolarPoint desired_pos = PolarPoint(
            facing_goal.elev,
            facing_goal.azim + 180.0f,
            this->config_.goal_min_dist
        );
        wrapPolar(desired_pos);
        this->goal_pos_ = convertPolarToCartesian(desired_pos, this->goal_);

        this->goal_updated_ = true;
        this->it_since_last_update_ = 0;
    }
    else
        this->goal_updated_ = false;
}


void LocalPlanner::setPointCloud(PointCloud<PointXYZ>& cloud) {
    this->cloud_cache_.clear();
    for (size_t i = 0; i < cloud.size(); i++)
        this->cloud_cache_.push_back(PointXYZ(cloud[i]));
    this->cloud_updated_ = true;
}


void LocalPlanner::run() {

    if (!this->goal_updated_) {
        this->goal_ = this->predict_next_target_();
        this->goal_updated_ = false;
        this->it_since_last_update_ += 1;
    }

    // away from goal position
    if ((this->goal_pos_-this->position_).norm() > this->config_.robot_pos_margin) {
        if (this->goal_updated_ || this->cloud_updated_)
            this->processPointCloud_();
        this->planNext_();
    }
    // within acceptable margin
    else {
        this->next_ = this->goal_pos_;
    }
}


Eigen::Vector3f LocalPlanner::getNext() const {
    return this->next_;
};


cv::Mat LocalPlanner::getHistogramImage() const {
    return this->image_histogram_;
}


cv::Mat LocalPlanner::getCostImage() const {
    return this->image_cost_;
}


void LocalPlanner::reset() {

    this->goal_ = Eigen::Vector3f::Zero();
    this->goal_pos_ = Eigen::Vector3f::Zero();;
    this->next_ = Eigen::Vector3f::Zero();;

    this->position_ = Eigen::Vector3f::Zero();
    this->orientation_ = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
    this->velocity_ = Eigen::Vector3f::Zero();
    this->prev_position_ = Eigen::Vector3f::Zero();

    this->fov_ = FOV(0.0f, 0.0f, this->config_.camera_fov_h, this->config_.camera_fov_v);

    this->state_updated_ = false;
    this->goal_updated_ = false;
    this->cloud_updated_ = false;

    this->cloud_cache_.clear();
    this->last_processing_time_ = std::chrono::system_clock::now();

    this->histogram_.clear();

    this->target_array_.clear();
    this->it_since_last_update_ = 0;
}


Eigen::Vector3f LocalPlanner::predict_next_target_() {

    Eigen::Vector3f latest_target = Eigen::Vector3f::Zero();
    if (this->target_array_.size() > 0)
        latest_target = this->target_array_[0];

    std::vector<Eigen::Vector3f> velocity_array = {};
    Eigen::Vector3f mean_velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    if (this->target_array_.size() > 1)
        for (unsigned i = 1; i < this->target_array_.size(); i++) {
            Eigen::Vector3f partial = Eigen::Vector3f(
                this->target_array_[i].x() - this->target_array_[i-1].x(),
                this->target_array_[i].y() - this->target_array_[i-1].y(),
                this->target_array_[i].z() - this->target_array_[i-1].z()
            );
            velocity_array.push_back(partial);
            mean_velocity += partial;
        }
    if (velocity_array.size() > 0)
        mean_velocity /= (float)velocity_array.size();

    std::vector<Eigen::Vector3f> acceleration_array = {};
    Eigen::Vector3f mean_acceleration = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    if (velocity_array.size() > 1)
        for (unsigned i = 1; i < velocity_array.size(); i++) {
            Eigen::Vector3f partial = Eigen::Vector3f(
                velocity_array[i].x() - velocity_array[i-1].x(),
                velocity_array[i].y() - velocity_array[i-1].y(),
                velocity_array[i].z() - velocity_array[i-1].z()
            );
            acceleration_array.push_back(partial);
            mean_acceleration += partial;
        }
    if (acceleration_array.size() > 0)
        mean_acceleration /= (float)acceleration_array.size();

    float t = this->config_.execution_time * (float)(1 + this->it_since_last_update_);
    Eigen::Vector3f predicted_target = latest_target + (mean_velocity * t) + (mean_acceleration * t * t * 0.5);

    return predicted_target;
}


void LocalPlanner::processPointCloud_() {

    int h_alpha = this->histogram_.getAlpha();
    int h_elev = this->histogram_.getElevRes();
    int h_azim = this->histogram_.getAzimRes();

    Eigen::MatrixXi counter(h_elev, h_azim);
    counter.fill(0.0f);

    PolarHistogram new_histogram = PolarHistogram(h_alpha);
    new_histogram.fillAge(INFINITY);

    float min_range_sq = sqr(this->config_.sensor_range_min);
    float max_range_sq = sqr(this->config_.sensor_range_max);

    std::chrono::duration<double> time_passed = std::chrono::system_clock::now() - this->last_processing_time_;
    double elapsed = time_passed.count();

    if (this->cloud_updated_) {
        if (this->config_.enable_cuda) {

        const float* k_point_cloud = (const float*)this->cloud_cache_.cloud_.data();
        this->kernels_->processIncomingPointCloud(k_point_cloud, this->cloud_cache_.size());
        float* k_distance = this->kernels_->getHistogramData();
        int* k_counter = this->kernels_->getCounterData();

        for (int elev = 0; elev < h_elev; elev++) {
            for (int azim = 0; azim < h_azim; azim++) {
                new_histogram.setDistance(elev, azim, k_distance[elev*h_azim+azim]);
                counter(elev, azim) = k_counter[elev*h_azim+azim];
            }
        }

        }
        else {

            for (const PointXYZ& point : this->cloud_cache_) {
                if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) continue;
                float distanceSq = sqr(point.x) + sqr(point.y) + sqr(point.z);
                if (min_range_sq <= distanceSq && distanceSq <= max_range_sq) {
                    PolarPoint polar = convertCartesianToPolar(toEigen(point));
                    // if (!pointInsideFOV(this->fov_, polar)) continue;
                    Eigen::Vector2i idx = convertPolarToHistogramIndex(polar, h_alpha);
                    counter(idx.y(), idx.x())++;
                    new_histogram.addToDistance(idx.y(), idx.x(), polar.radi);
                    // new_histogram.setAge(idx.y(), idx.x(), 0.0f); // unnecessary since 'new_histogram' is initialized with '0'
                }
            }

        }
    }

    for (int elev = 0; elev < h_elev; elev++) {
        for (int azim = 0; azim < h_azim; azim++) {
            if (this->histogram_.getDistance(elev, azim) && this->histogram_.getAge(elev, azim) < this->config_.point_max_age) {
                PolarPoint polar = convertHistogramIndexToPolar(elev, azim, h_alpha, this->histogram_.getDistance(elev, azim));
                PolarPoint tmp_polar, new_polar;
                for (float elev_step = -h_alpha/2; elev_step < h_alpha; elev_step += h_alpha) {
                    for (float azim_step = -h_alpha/2; azim_step < h_alpha; azim_step += h_alpha) {
                        tmp_polar = PolarPoint(polar.elev + elev_step, polar.azim + azim_step, polar.radi);
                        new_polar = convertCartesianToPolar(convertPolarToCartesian(new_polar, this->prev_position_), this->position_);
                        if (sqr(new_polar.radi) < max_range_sq && (!this->cloud_updated_ || !pointInsideFOV(this->fov_, new_polar)) ) {
                            Eigen::Vector2i idx = convertPolarToHistogramIndex(new_polar, h_alpha);
                            counter(idx.y(), idx.x())++;
                            new_histogram.addToDistance(idx.y(), idx.x(), new_polar.radi);
                            new_histogram.setAge(idx.y(), idx.x(), std::min(new_histogram.getAge(idx.y(), idx.x()), this->histogram_.getAge(elev, azim)));
                        }
                    }
                }

            }
        }
    }

    Eigen::Vector2i h_lim_epap = convertAngleToHistogramIndex(this->fov_.pitch_deg + this->fov_.v_fov_deg, this->fov_.yaw_deg + this->fov_.h_fov_deg, h_alpha);
    Eigen::Vector2i h_lim_enan = convertAngleToHistogramIndex(this->fov_.pitch_deg - this->fov_.v_fov_deg, this->fov_.yaw_deg - this->fov_.h_fov_deg, h_alpha);

    for (int elev = 0; elev < h_elev; elev++) {
        for (int azim = 0; azim < h_azim; azim++) {
        if (this->cloud_updated_ && elev <= h_lim_epap.y() && elev >= h_lim_enan.y() && azim <= h_lim_epap.x() && azim >= h_lim_enan.x()) {
            if (counter(elev, azim) > 0)
                new_histogram.setCell(elev, azim, new_histogram.getDistance(elev, azim) / counter(elev, azim), 0.0f);
            else
                new_histogram.setCell(elev, azim, 0.0f, 0.0f);
        }
        else {
            if (counter(elev, azim) > 0)
                new_histogram.setCell(elev, azim, new_histogram.getDistance(elev, azim) / counter(elev, azim), new_histogram.getAge(elev, azim) + elapsed);
            else
                new_histogram.setCell(elev, azim, 0.0f, 0.0f);
        }
        }
    }

    this->histogram_ = new_histogram;
    this->generateHistogramImage_(this->histogram_, this->image_histogram_);
    
    this->last_processing_time_ = std::chrono::system_clock::now();
    this->cloud_updated_ = false;
}


CostFunctionOutput LocalPlanner::costFunction_(
    const PolarPoint& candidate,
    const Eigen::Vector3f& position,
    const Eigen::Vector3f& velocity, 
    float obstacle_distance
) const {

    float d = this->config_.obstacle_distance_min - obstacle_distance;
    float distance_cost = obstacle_distance > 0.0f ? this->config_.cost_obstacle_distance * (1 + d / std::sqrt(1 + d * d)) : 0.0f;

    Eigen::Vector3f candidate_velocity_cartesian = convertPolarToCartesian(candidate);
    float velocity_cost = this->config_.cost_velocity * (velocity.norm() - candidate_velocity_cartesian.normalized().dot(velocity));

    PolarPoint facing_goal = convertCartesianToPolar(this->goal_pos_, position);
    float angle_diff = angleDifference(candidate.azim, facing_goal.azim);
    float yaw_cost = this->config_.cost_yaw * sqr(angle_diff);

    return CostFunctionOutput(distance_cost, velocity_cost + yaw_cost);
}


void LocalPlanner::getCostMatrix_(
    const PolarHistogram& histogram,
    const Eigen::Vector3f& position,
    const Eigen::Vector3f& velocity,
    Eigen::MatrixXf& cost_matrix,
    cv::Mat& cost_image
) const {

    Eigen::MatrixXf distance_matrix(histogram.getElevRes(), histogram.getAzimRes());
    distance_matrix.fill(NAN);

    cost_matrix.resize(histogram.getElevRes(), histogram.getAzimRes());
    cost_matrix.fill(NAN);

    // Fill in cost matrix
    for (int elev = 0; elev < histogram.getElevRes(); elev++) {
        // Determine how many bins at this elevation angle would be equivalent 
        // to a single bin at horizontal, then work in steps of that size
        const float bin_width = std::cos(convertHistogramIndexToPolar(elev, 0, histogram.getAlpha(), 1.0f).elev * DEG_TO_RAD);
        const int step_size = static_cast<int>(std::round(1.0f / bin_width));

        for (int azim = 0; azim < histogram.getAzimRes(); azim += step_size) {
            float obstacle_distance = histogram.getDistance(elev, azim);
            PolarPoint polar = convertHistogramIndexToPolar(elev, azim, histogram.getAlpha(), 1.0f); // unit vector of current direction
            CostFunctionOutput costs = costFunction_(polar, position, velocity, obstacle_distance);
            distance_matrix(elev, azim) = costs.distance_cost;
            cost_matrix(elev, azim) = costs.state_cost;
        }
        if (step_size > 1) {
            // Horizontally interpolate all of the un-calculated values
            int last_index = 0;
            for (int azim = step_size; azim < histogram.getAzimRes(); azim += step_size) {
                float distance_cost_gradient = (distance_matrix(elev, azim) - distance_matrix(elev, last_index)) / step_size;
                float other_costs_gradient   = (cost_matrix(elev, azim) - cost_matrix(elev, last_index)) / step_size;
                for (int i = 1; i < step_size; i++) {
                    distance_matrix(elev, last_index + i) = distance_matrix(elev, last_index) + distance_cost_gradient * i;
                    cost_matrix(elev, last_index + i)     = cost_matrix(elev, last_index) + other_costs_gradient * i;
                }
                last_index = azim;
            }

            // Special case the last columns wrapping around back to 0
            int clamped_z_scale = histogram.getAzimRes() - last_index;
            float distance_cost_gradient = (distance_matrix(elev, 0) - distance_matrix(elev, last_index)) / clamped_z_scale;
            float other_costs_gradient   = (cost_matrix(elev, 0) - cost_matrix(elev, last_index)) / clamped_z_scale;
            for (int i = 1; i < clamped_z_scale; i++) {
                distance_matrix(elev, last_index + i) = distance_matrix(elev, last_index) + distance_cost_gradient * i;
                cost_matrix(elev, last_index + i)     = cost_matrix(elev, last_index) + other_costs_gradient * i;
            }
        }
    }

    cost_matrix += distance_matrix;

    // calculate mean
    for (int elev = 0; elev < histogram.getElevRes(); elev++) {
        for (int azim = 0; azim < histogram.getAzimRes(); azim ++) {
            float mean = 0.0f;
            for (int i = -2; i < 3; i++) {
                int y = elev, x = azim+i;
                histogram.wrapIndex(y, x);
                mean += cost_matrix(y, x);
            }
            mean /= 5;
            cost_matrix(elev, azim) = mean;
        }
    }

    this->generateCostImage_(cost_matrix, distance_matrix, cost_image);
}


void LocalPlanner::getBestMoveDirections_(const Eigen::MatrixXf& cost_matrix, std::vector<MoveDirection>& direction_list) const {
    direction_list.clear();
    for (int i = 0; i < cost_matrix.rows(); i++) {
        for (int j = 0; j < cost_matrix.cols(); j++) {
            PolarPoint polar = convertHistogramIndexToPolar(i, j, this->config_.alpha, 1.0);
            MoveDirection candidate(polar.elev, polar.azim, cost_matrix(i, j));

            unsigned it = 0;
            while (it < direction_list.size() && candidate > direction_list[it]) it++;
            direction_list.insert(std::next(direction_list.begin(), it), candidate);
            if (direction_list.size() > this->config_.max_candidates_per_it) direction_list.pop_back();
        }
    }

    // @TODO get best direction candidates based on future predicted goal
}


void LocalPlanner::planNext_() {

    if (this->config_.skip_planning) {
        this->next_ = this->goal_pos_;
        return;
    }

    Eigen::MatrixXf cost_matrix;
    cv::Mat cost_image(this->histogram_.getElevRes(), this->histogram_.getAzimRes(), CV_8UC3, cv::Scalar(0,0,0));;
    std::vector<MoveDirection> direction_list;
    direction_list.clear();
    
    this->getCostMatrix_(this->histogram_, this->position_, this->velocity_, cost_matrix, cost_image);
    this->getBestMoveDirections_(cost_matrix, direction_list);

    MoveDirection best_move = direction_list[0];

    // @TODO solve case when goal pos within planning_step, but move direction not towards the goal pos
    float next_pos_dist = (this->position_ - this->goal_pos_).norm();
    float step_size = next_pos_dist > this->config_.planning_step ? this->config_.planning_step : next_pos_dist;

    this->next_ = convertPolarToCartesian(PolarPoint(best_move.elevation, best_move.azimuth, step_size), this->position_);

    this->image_cost_ = cost_image;
}


void LocalPlanner::generateHistogramImage_(
    const PolarHistogram& histogram,
    cv::Mat& image_data
) const {

    float max_val = this->config_.sensor_range_max;

    for (int e = this->histogram_.getElevRes() - 1; e >= 0; e--) {
        for (int z = this->histogram_.getAzimRes() - 1; z >=0; z--) {
            float distance = histogram.getDistance(e, z);
            float distance_display = distance > 0.0f ? 255.0f * (1.0f - (distance / max_val)) : 0.0f;

            cv::Vec3b& pixel = image_data.at<cv::Vec3b>(e, z);
            pixel[0] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, distance_display)));
            pixel[1] = 0;
            pixel[2] = 0;
        }
    }
}


void LocalPlanner::generateCostImage_(
    const Eigen::MatrixXf& cost_matrix,
    const Eigen::MatrixXf& distance_matrix,
    cv::Mat& image_data
) const {

    float max_val = std::max(cost_matrix.maxCoeff(), distance_matrix.maxCoeff());

    for (int e = this->histogram_.getElevRes() - 1; e >= 0; e--) {
        for (int z = this->histogram_.getAzimRes() - 1; z >=0; z--) {
            float distance_cost = 255.0f * distance_matrix(e, z) / max_val;
            float other_cost = 255.0f * cost_matrix(e, z) / max_val;

            cv::Vec3b& pixel = image_data.at<cv::Vec3b>(e, z);
            pixel[0] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, distance_cost)));
            pixel[1] = static_cast<uint8_t>(std::max(0.0f, std::min(255.0f, other_cost)));
            pixel[2] = 0;
        }
    }
}


} // namespace NAVIGATION_CORE
