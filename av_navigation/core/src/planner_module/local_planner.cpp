#include "local_planner.hpp"


namespace NAVIGATION_CORE {

LocalPlanner::LocalPlanner() {
    this->initialize();
}

LocalPlanner::LocalPlanner(LocalPlannerConfig config) : config_(config) {
    this->initialize();
}

void LocalPlanner::initialize() {
    this->fov_ = FOV(0.0f, 0.0f, this->config_.camera_fov_h, this->config_.camera_fov_v);
    this->translation_flatten_[0]  = 1.0f; // [1,1]
    this->translation_flatten_[5]  = 1.0f; // [2,2]
    this->translation_flatten_[10] = 1.0f; // [3,3]
    this->last_processing_time_ = std::chrono::system_clock::now();

    this->drone_ready_ = false;

    this->histogram_ = PolarHistogram(this->config_.alpha);
    this->hist_image_ = cv::Mat(this->histogram_.getElevRes(), this->histogram_.getAzimRes(), CV_8UC3, cv::Scalar(0,0,0));
    this->cost_image_ = cv::Mat(this->histogram_.getElevRes(), this->histogram_.getAzimRes(), CV_8UC3, cv::Scalar(0,0,0));

    NAVIGATION_CORE_KERNELS::KernelsConfig k_config;
    k_config.alpha = this->config_.alpha;
    k_config.elev_res = this->histogram_.getElevRes();
    k_config.azim_res = this->histogram_.getAzimRes();
    k_config.flat_size = k_config.elev_res * k_config.azim_res;
    k_config.min_distance = this->config_.sensor_min_range;
    k_config.max_distance = this->config_.sensor_max_range;
    k_config.max_age = this->config_.point_max_age;
    this->kernels_ = std::make_unique<NAVIGATION_CORE_KERNELS::LocalPlannerKernels>(k_config);
}

void LocalPlanner::setState(Eigen::Vector3f position, Eigen::Vector3f orientation, Eigen::Vector3f velocity) {

    this->prev_position_ = this->position_;

    this->position_     = position;
    this->orientation_  = orientation;
    this->lin_velocity_ = velocity;

    float cr = 20.0f / 180.0f * PI_F;

    Eigen::Quaternionf rotation = Eigen::AngleAxisf(this->orientation_.z(), Eigen::Vector3f::UnitZ()) *
                                  Eigen::AngleAxisf(this->orientation_.y() - cr, Eigen::Vector3f::UnitY()) *
                                  Eigen::AngleAxisf(this->orientation_.x(), Eigen::Vector3f::UnitX());

    this->rotation_matrix_ = rotation.normalized().toRotationMatrix();
  
    for (short i=0; i<3; i++) {
        for (short j=0; j<3; j++)
            this->translation_flatten_[i*4+j] = this->rotation_matrix_(i,j);
        this->translation_flatten_[i*4+3] = this->position_[i];
    }

    this->fov_.yaw_deg = this->orientation_.z() * RAD_TO_DEG;
    this->fov_.pitch_deg = this->orientation_.y() * RAD_TO_DEG;

    this->state_updated_ = true;
}

void LocalPlanner::setGoal(Eigen::Vector3f goal) {
    if (std::isnan(goal.x()) || std::isnan(goal.y()) || std::isnan(goal.z())) {
        // return;
        goal = this->goal_pred_;
    }

    Eigen::Vector3f new_goal = this->rotation_matrix_ * goal + this->position_;
    
    if ((this->goal_ - new_goal).norm() > this->config_.goal_dev_margin) {

        this->prev_goal_array_.push_back(new_goal);
        if (this->prev_goal_array_.size() > this->config_.prev_goal_num)
            this->prev_goal_array_ = std::vector<Eigen::Vector3f>(this->prev_goal_array_.begin()+1, this->prev_goal_array_.end());
        this->predictNewGoal();
    
        this->goal_ = new_goal;

        PolarPoint facing_goal = convertCartesianToPolar(this->goal_, this->position_);
        PolarPoint desired_pos = PolarPoint(std::asin(this->config_.goal_min_alt_diff / this->config_.goal_min_dist) * RAD_TO_DEG,
                                            facing_goal.azim + 180.0f, 
                                            this->config_.goal_min_dist);
        wrapPolar(desired_pos);
        this->goal_pos_ = convertPolarToCartesian(desired_pos, this->goal_);

        this->goal_updated_ = true;
    }
    else
        this->goal_updated_ = false;
}

void LocalPlanner::setPointCloud(const PointCloud<PointXYZ>& cloud) {
    this->cloud_cache_.clear();
    for (auto &point : cloud) {
        this->cloud_cache_.push_back(this->transformPoint(point));
    }
    this->cloud_updated_ = true;
}

void LocalPlanner::run() {

    if (this->drone_ready_) {
        // Drone away from final goal
        if ((this->goal_pos_-this->position_).norm() > this->config_.drone_pos_margin) {
            if (this->goal_updated_ || this->cloud_updated_) {
                this->processPointCloud();
            }
            this->planNext();
        }
        // Drone within acceptable margin
        else {
            this->next_ = this->goal_pos_;
        }
    }
    else {
        this->next_ = Eigen::Vector3f(0.0f, 0.0f, this->config_.init_altitude);
        if (this->position_.z() > this->config_.init_altitude-0.1f)
            this->drone_ready_ = true;
    }
}

Eigen::Vector3f LocalPlanner::getNext() const {
    return this->next_;
};

PointXYZ LocalPlanner::transformPoint(PointXYZ point) const {
    // Eigen::Vector3f rotated = this->rotation_matrix_ * Eigen::Vector3f(point.x, point.y, point.z) + this->position_;
    // return PointXYZ(rotated.x(), rotated.y(), rotated.z());

    float x = point.x * this->translation_flatten_[0] +
              point.y * this->translation_flatten_[1] +
              point.z * this->translation_flatten_[2] +
              this->translation_flatten_[3];
    float y = point.x * this->translation_flatten_[4] +
              point.y * this->translation_flatten_[5] +
              point.z * this->translation_flatten_[6] +
              this->translation_flatten_[7];
    float z = point.x * this->translation_flatten_[8] +
              point.y * this->translation_flatten_[9] +
              point.z * this->translation_flatten_[10] +
              this->translation_flatten_[11];
    return PointXYZ(x, y, z);
}

void LocalPlanner::predictNewGoal() {

    if (this->prev_goal_array_.size() < this->config_.prev_goal_num) return;

    static std::chrono::system_clock::time_point last_pred;

    double dt = (std::chrono::system_clock::now() - last_pred).count();

    std::vector<Eigen::Vector3f> velArr;
    Eigen::Vector3f velMean = Eigen::Vector3f::Zero();
    for (unsigned i = 0; i < this->config_.prev_goal_num-1; i++) {
        Eigen::Vector3f vel = this->prev_goal_array_[i+1] - this->prev_goal_array_[i];
        velArr.push_back(vel);
        velMean += vel;
    }
    velMean /= velArr.size();

    std::vector<Eigen::Vector3f> accArr;
    Eigen::Vector3f accMean = Eigen::Vector3f::Zero();
    for (unsigned i = 0; i < this->config_.prev_goal_num-2; i++) {
        Eigen::Vector3f acc = velArr[i+1] - velArr[i];
        accArr.push_back(acc);
        accMean += acc;
    }
    accMean /= accArr.size();

    this->goal_pred_ = this->prev_goal_array_[this->config_.prev_goal_num-1] + velMean * dt + 0.5 * accMean * dt * dt;

    last_pred = std::chrono::system_clock::now();
}

void LocalPlanner::processPointCloud() {

    int h_alpha = this->histogram_.getAlpha();
    int h_elev = this->histogram_.getElevRes();
    int h_azim = this->histogram_.getAzimRes();

    Eigen::MatrixXi counter(h_elev, h_azim);
    counter.fill(0.0f);

    PolarHistogram new_histogram = PolarHistogram(h_alpha);
    new_histogram.fillAge(INFINITY);

    float min_range_sq = sqr(this->config_.sensor_min_range);
    float max_range_sq = sqr(this->config_.sensor_max_range);

    std::chrono::duration<double> time_passed = std::chrono::system_clock::now() - this->last_processing_time_;
    double elapsed = time_passed.count();

    if (this->cloud_updated_) {
        if (this->config_.en_cuda) {

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
    this->generateHistImage(this->histogram_, this->hist_image_);
    
    this->last_processing_time_ = std::chrono::system_clock::now();
    this->cloud_updated_ = false;
}

CostFunctionOutput LocalPlanner::costFunction(
    const PolarPoint& candidate,
    const Eigen::Vector3f& position,
    const Eigen::Vector3f& velocity, 
    float obstacle_distance) const
{
    PolarPoint facing_goal = convertCartesianToPolar(this->goal_pos_, position);
    float angle_diff = angleDifference(candidate.azim, facing_goal.azim);

    float goal_distance = (this->goal_pos_ - position).norm();
    Eigen::Vector3f candidate_velocity_cartesian = convertPolarToCartesian(candidate);

    float velocity_cost = this->config_.velocity_cost_param * (velocity.norm() - candidate_velocity_cartesian.normalized().dot(velocity));

    float yaw_cost   = this->config_.yaw_cost_param * sqr(angle_diff);
    float pitch_cost = this->config_.pitch_cost_param * sqr(candidate.elev - facing_goal.elev);

    // Increase the pitch cost starting at 5m from the goal (forcing the drone to goal altitude)
    if (goal_distance < this->config_.pitch_block_distance)
        pitch_cost = pitch_cost * sqr(this->config_.pitch_block_distance) / sqr(goal_distance);

    float d = this->config_.obstacle_min_distance - obstacle_distance;
    float distance_cost = obstacle_distance > 0.0f ? this->config_.obstacle_cost_param * (1 + d / std::sqrt(1 + d * d)) : 0.0f;

    return CostFunctionOutput(distance_cost, velocity_cost + yaw_cost + pitch_cost);
}

void LocalPlanner::getCostMatrix(
    const PolarHistogram& histogram,
    const Eigen::Vector3f& position,
    const Eigen::Vector3f& velocity,
    Eigen::MatrixXf& cost_matrix,
    cv::Mat& cost_image) const
{
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
            CostFunctionOutput costs = costFunction(polar, position, velocity, obstacle_distance);
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

    this->generateCostImage(cost_matrix, distance_matrix, cost_image);
}

void LocalPlanner::getBestMoveDirections(const Eigen::MatrixXf& cost_matrix, std::vector<MoveDirection>& direction_list) const {
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

void LocalPlanner::planNext() {

    if (this->config_.skip_planning) {
        this->next_ = this->goal_pos_;
        return;
    }

    Eigen::MatrixXf cost_matrix;
    cv::Mat cost_image(this->histogram_.getElevRes(), this->histogram_.getAzimRes(), CV_8UC3, cv::Scalar(0,0,0));;
    std::vector<MoveDirection> direction_list;
    direction_list.clear();
    
    this->getCostMatrix(this->histogram_, this->position_, this->lin_velocity_, cost_matrix, cost_image);
    this->getBestMoveDirections(cost_matrix, direction_list);

    MoveDirection best_move = direction_list[0];

    // @TODO solve case when goal pos within planning_step, but move direction not towards the goal pos
    float next_pos_dist = (this->position_ - this->goal_pos_).norm();
    float step_size = next_pos_dist > this->config_.planning_step ? this->config_.planning_step : next_pos_dist;

    this->next_ = convertPolarToCartesian(PolarPoint(best_move.elevation, best_move.azimuth, step_size), this->position_);

    this->cost_image_ = cost_image;
}

void LocalPlanner::reset() {

    this->state_updated_ = false;
    this->goal_updated_ = false;
    this->cloud_updated_ = false;

    this->prev_goal_array_.clear();
    this->extr_goal_array_.clear();

    this->last_processing_time_ = std::chrono::system_clock::now();
}

void LocalPlanner::generateHistImage(
    const PolarHistogram& histogram,
    cv::Mat& image_data) const 
{
    float max_val = this->config_.sensor_max_range;

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

void LocalPlanner::generateCostImage(
    const Eigen::MatrixXf& cost_matrix,
    const Eigen::MatrixXf& distance_matrix,
    cv::Mat& image_data) const 
{
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

cv::Mat LocalPlanner::getHistImage() const {
    return this->hist_image_;
}

cv::Mat LocalPlanner::getCostImage() const {
    return this->cost_image_;
}

}  // namespace NAVIGATION_CORE
