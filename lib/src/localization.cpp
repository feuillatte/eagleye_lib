#include "localization.hpp"

#include <navigation/navigation.hpp>

EaglEyeLocalization::EaglEyeLocalization(EaglEyeParameters p) {
    velocity_enable_status_.status.enabled_status = true; // HACK

    velocity_scale_factor_parameter_.imu_rate = p.imu_rate_hz;
    velocity_scale_factor_parameter_.gnss_rate =p.gnss_rate_hz;
    velocity_scale_factor_parameter_.moving_judgement_threshold = p.vehicle_moving_judgement_threshold_speed_mps;
    velocity_scale_factor_parameter_.estimated_minimum_interval = p.velocity_scale_factor_buffering_init_time_min_s;
    velocity_scale_factor_parameter_.estimated_maximum_interval = p.velocity_scale_factor_buffering_init_time_max_s;
    velocity_scale_factor_parameter_.gnss_receiving_threshold = p.velocity_scale_factor_minimum_gnss_rate;
    velocity_scale_factor_parameter_.save_velocity_scale_factor = p.velocity_scale_factor_storage;

    position_parameter_.imu_rate = p.imu_rate_hz;
    position_parameter_.gnss_rate = p.gnss_rate_hz;
    position_parameter_.moving_judgement_threshold = p.vehicle_moving_judgement_threshold_speed_mps;
    position_parameter_.estimated_interval = p.position_data_buffering_interval_m;
    position_parameter_.update_distance = p.position_data_buffer_update_minimum_distance_m;
    position_parameter_.gnss_receiving_threshold = p.position_gnss_reception_threshold;
    position_parameter_.outlier_threshold = p.position_outlier_threshold_m;
    position_parameter_.outlier_ratio_threshold = p.position_outlier_threshold_ratio;
    position_parameter_.gnss_error_covariance = 0.5F;  // TODO FIXME HACK

    position_interpolate_parameter_.imu_rate = p.imu_rate_hz;
    position_interpolate_parameter_.stop_judgement_threshold = p.vehicle_stop_judgement_threshold_speed_mps;
    position_interpolate_parameter_.sync_search_period = p.position_interpolation_sync_search_period_s;
    position_interpolate_parameter_.proc_noise = p.position_interpolation_process_noise;

    slip_angle_parameter_.manual_coefficient = p.slip_angle_manual_coefficient;

    heading_interpolate_parameter_.imu_rate = p.imu_rate_hz;
    heading_interpolate_parameter_.stop_judgement_threshold = p.vehicle_stop_judgement_threshold_speed_mps;
    heading_interpolate_parameter_.sync_search_period = p.heading_interpolation_sync_search_period_s;
    heading_interpolate_parameter_.proc_noise = p.heading_interpolation_process_noise;

    heading_parameter_.imu_rate = p.imu_rate_hz;
    heading_parameter_.gnss_rate = p.gnss_rate_hz;
    heading_parameter_.stop_judgement_threshold = p.vehicle_stop_judgement_threshold_speed_mps;
    heading_parameter_.moving_judgement_threshold = p.vehicle_moving_judgement_threshold_speed_mps;
    heading_parameter_.estimated_minimum_interval = p.heading_data_buffering_init_time_min_s;
    heading_parameter_.estimated_maximum_interval = p.heading_data_buffering_init_time_max_s;
    heading_parameter_.gnss_receiving_threshold = p.heading_minimum_gnss_rate;
    heading_parameter_.outlier_threshold = p.heading_outlier_threshold;
    heading_parameter_.outlier_ratio_threshold = p.heading_outlier_threshold_ratio;
    heading_parameter_.curve_judgement_threshold = p.heading_curve_judgement_threshold_rps;
    heading_parameter_.init_STD = p.heading_init_stddev;

    yaw_rate_offset_parameter_.imu_rate = p.imu_rate_hz;
    yaw_rate_offset_parameter_.gnss_rate = p.gnss_rate_hz;
    yaw_rate_offset_parameter_.moving_judgement_threshold = p.vehicle_moving_judgement_threshold_speed_mps;
    yaw_rate_offset_parameter_.estimated_minimum_interval = p.yaw_rate_offset_buffering_init_time_min_s;
    yaw_rate_offset_parameter_.estimated_maximum_interval = p.yaw_rate_offset_buffering_init_time_max_1st_s;
    yaw_rate_offset_parameter_.gnss_receiving_threshold = p.yaw_rate_offset_minimum_gnss_rate;
    yaw_rate_offset_parameter_.outlier_threshold = p.yaw_rate_offset_outlier_threshold;

    yaw_rate_offset_stop_parameter_.imu_rate = p.imu_rate_hz;
    yaw_rate_offset_stop_parameter_.estimated_interval = p.yaw_rate_offset_stop_buffering_init_time_min_s;
    yaw_rate_offset_stop_parameter_.stop_judgement_threshold = p.vehicle_stop_judgement_threshold_speed_mps;
    yaw_rate_offset_stop_parameter_.outlier_threshold = p.yaw_rate_offset_stop_outlier_threshold;
}

void EaglEyeLocalization::addImuMeasurement(const ImuState& I) {
    last_imu_data_ = I;
    if (acc_x_offset_.status.enabled_status && acc_x_scale_factor_.status.enabled_status) {
        last_imu_data_.linear_acceleration_mpss.x = I.linear_acceleration_mpss.x * acc_x_scale_factor_.acc_x_scale_factor + acc_x_offset_.acc_x_offset;
    }
    // Some kind of coordinate system conversion. Reason unknown.
    last_imu_data_.angular_velocity_rps.z = -1.F * (I.angular_velocity_rps.z + angular_velocity_offset_stop_.angular_velocity_offset.z);
    has_new_imu_data_ = true;
}

void EaglEyeLocalization::addGnssPvt(const PositionVelocityTimeSolution& G) {
    last_gnss_.timestamp_ns = G.timestamp_ns;
    last_gnss_.position = G.position;
    enu_velocities_.header.stamp.tv_sec = G.timestamp_ns / 1e9;
    enu_velocities_.header.stamp.tv_nsec = G.timestamp_ns - enu_velocities_.header.stamp.tv_sec * 1e9;
    // Convert froM NED to ENU
    enu_velocities_.vector.x = G.velocity_ned_ms.y;
    enu_velocities_.vector.y = G.velocity_ned_ms.x;
    enu_velocities_.vector.z = - G.velocity_ned_ms.z;  // Note the flip
    has_new_gnss_data_ = true;
}

void EaglEyeLocalization::addWheelSpeeds(const WheelSpeedMeasurement& ws) {
    // Select the wheel speed measurement based on the vehicle powertrain configuration.
    // The driven (as in unpowered) wheels are a better measure for linear forward speed, because
    // the driving (powered) wheels have a natural tendency to slip when the torque exceeds
    // the friction of the wheel to the ground (for whatever reason), causing misleading measurements.
    float driven_wheel_speed_average_mps;
    if (config_.drive_type == EaglEyeParameters::DriveType::FWD) {  // AWD also when in FWD mode
        driven_wheel_speed_average_mps = 0.5F * (ws.wheelspeed_rr_mps + ws.wheelspeed_rl_mps);
    } else if (config_.drive_type == EaglEyeParameters::DriveType::RWD) {
        driven_wheel_speed_average_mps = 0.5F * (ws.wheelspeed_fr_mps + ws.wheelspeed_fl_mps);
    } else {  // 4WD: All wheels are driving wheels, so we need to get a best guess at the average
        driven_wheel_speed_average_mps =
            0.25F *
            (ws.wheelspeed_fr_mps + ws.wheelspeed_fl_mps +
             ws.wheelspeed_rr_mps + ws.wheelspeed_rl_mps);
    }
    last_velocities_.header.stamp.tv_sec = ws.timestamp_ns / 1e9;
    last_velocities_.header.stamp.tv_nsec = ws.timestamp_ns - last_velocities_.header.stamp.tv_sec * 1e9;
    last_velocities_.twist.linear.x = driven_wheel_speed_average_mps;
    has_new_wheelspeed_data_ = true;
    if (vehicle_has_moved_ == false && last_velocities_.twist.linear.x > config_.vehicle_moving_judgement_threshold_speed_mps) {
        vehicle_has_moved_ = true;
    }
}

void EaglEyeLocalization::addSteeringAngleMeasurement() {}

// The order of evaluation is assumed to be important, as the underlying logic in eagleye_core appears
// to assume a certain order of internal state varaible updates to properly propagate data forward in
// the implcit computation graph. The assumed order is as follows:
// 0. yaw_rate_offset_stop_estimate() | velocity_scale_factor_estimate()
// 1. slip_angle_estimate() | distance_estimate()
// 2. heading_interpolate_3rd() | pitching_estimate()
// 3. trajectory3d_estimate()
// 4. position_estimate()
// 5. position_interpolate()
//
void EaglEyeLocalization::computeState() {
    if (vehicle_has_moved_ == false) {
        velocity_scale_factor_ = default_velocity_scale_factor_;
    } else {
        TwistStamped corrected_velocity{};
        velocity_scale_factor_estimate(
            last_pvt_,
            last_velocities_,
            velocity_scale_factor_parameter_,
            &velocity_scale_factor_status_,
            &corrected_velocity,
            &velocity_scale_factor_
        );
        if (std::isfinite(velocity_scale_factor_.scale_factor) == false) {
            corrected_velocity.twist.linear.x  = last_velocities_.twist.linear.x * previous_velocity_scale_factor_;
            velocity_scale_factor_.scale_factor = previous_velocity_scale_factor_;
            velocity_scale_factor_.status.is_abnormal = true;
            velocity_scale_factor_.status.error_code = Status::NAN_OR_INFINITE;
        } else if (config_.global_velocity_scale_factor_percent / 100 < std::abs(1.0 - velocity_scale_factor_.scale_factor)) {
            corrected_velocity.twist.linear.x = last_velocities_.twist.linear.x * previous_velocity_scale_factor_;
            velocity_scale_factor_.scale_factor = previous_velocity_scale_factor_;
            velocity_scale_factor_.status.is_abnormal = true;
            velocity_scale_factor_.status.error_code = Status::TOO_LARGE_OR_SMALL;
        } else {
            previous_velocity_scale_factor_ = velocity_scale_factor_.scale_factor;
        }
        last_velocities_ = corrected_velocity;
    }
    if (has_new_imu_data_) {
        has_new_imu_data_ = false;

        yaw_rate_offset_stop_estimate(
            last_velocities_,
            last_imu_data_,
            yaw_rate_offset_stop_parameter_,
            &yaw_rate_offset_stop_status_,
            &yaw_rate_offset_stop_
        );
        // The below postprocessing is adapted from eagleye_rt/src/yaw_rate_offset_stop_node.cpp
        if (!std::isfinite(yaw_rate_offset_stop_.yaw_rate_offset)) {
            yaw_rate_offset_stop_.yaw_rate_offset = previous_yaw_rate_offset_stop_;
            yaw_rate_offset_stop_.status.is_abnormal = true;
            yaw_rate_offset_stop_.status.error_code = Status::NAN_OR_INFINITE;
        } else {
            previous_yaw_rate_offset_stop_ = yaw_rate_offset_stop_.yaw_rate_offset;
            yaw_rate_offset_stop_.status.is_abnormal = false;
        }

        // Prepare slip angle estimation
        velocity_enable_status_.header = velocity_scale_factor_.header;
        velocity_enable_status_.status = velocity_scale_factor_.status;
        estimated_slip_angle_.header.stamp.tv_sec = last_imu_data_.timestamp_ns / 1e9;
        estimated_slip_angle_.header.stamp.tv_nsec = last_imu_data_.timestamp_ns - estimated_slip_angle_.header.stamp.tv_sec * 1e9;
        slip_angle_estimate(  // Estimate the wheel slip angle (actual motion angle relative to wheel angle)
            last_imu_data_,
            last_velocities_,
            velocity_enable_status_,
            yaw_rate_offset_stop_,
            yaw_rate_offset_2nd_,
            slip_angle_parameter_,
            &estimated_slip_angle_
        );

        pitching_estimate(  // Estimate the pitch
            last_imu_data_,
            last_gnss_,
            last_velocities_,
            distance_estimate_,
            height_parameter_,
            &height_status_,
            &estimated_height_,
            &estimated_pitch_,
            &acc_x_offset_,
            &acc_x_scale_factor_
        );
        rolling_estimate(  // Estimate the roll
            last_imu_data_,
            last_velocities_,
            yaw_rate_offset_stop_,
            yaw_rate_offset_2nd_,
            rolling_parameter_,
            &rolling_status_,
            &estimated_roll_
        );
    }
    if (has_new_wheelspeed_data_) {
        has_new_wheelspeed_data_ = false;
        distance_estimate(
            last_velocities_,
            &distance_status_,
            &distance_estimate_
        );
    }
    if (has_new_steering_data_) {
        has_new_steering_data_ = false;
        const float R{config_.wheel_base_m / std::tan(last_steer_angle_rad_ * config_.steer_ratio)};
        const float W{last_velocities_.twist.linear.x / R};
        last_velocities_.twist.angular.z = W;
    }

    trajectory3d_estimate(
        last_imu_data_,
        last_velocities_,
        velocity_enable_status_,
        estimated_heading_,
        yaw_rate_offset_stop_,
        yaw_rate_offset_2nd_,
        estimated_pitch_,
        trajectory_parameter_,
        &trajectory_status_,
        &enu_velocities_,
        &estimated_position_,
        &last_velocities_,
        &last_velocities_with_cov_
    );
    position_estimate(  // Updates estimated position
        last_gnss_,
        last_velocities_,
        velocity_enable_status_,
        distance_estimate_,
        estimated_heading_,
        enu_velocities_,
        position_parameter_,
        &position_status_,
        &estimated_position_
    );
    position_interpolate_estimate(
        estimated_position_,
        enu_velocities_,
        estimated_position_,   // should be Smoothed GNSS position instead
        estimated_height_,
        estimated_heading_,
        position_interpolate_parameter_,
        &position_interpolate_status_,
        &estimated_position_,
        &estimated_llh_
    );
    estimated_position_.header = enu_velocities_.header;
}

void EaglEyeLocalization::resetRelativeMotionTrackingOrigin(const GNSSPosition& p) {
    llh2xyz(reinterpret_cast<double*>(const_cast<GNSSPosition*>(&p)), reinterpret_cast<double*>(&estimated_position_.ecef_base_pos));
}

GNSSState EaglEyeLocalization::getGlobalPoseStateLLA() {
    GNSSState g;
    g.timestamp_ns = estimated_position_.header.stamp.tv_sec * 1e9 + estimated_position_.header.stamp.tv_nsec;
    g.position.lat = estimated_llh_.lat;
    g.position.lon = estimated_llh_.lon;
    g.position.alt_msl = estimated_llh_.alt_msl;
    g.position.surface_stddev_m = estimated_position_.covariance[4];
    g.position.alt_stddev_m = estimated_position_.covariance[8];
    return g;
}

Position EaglEyeLocalization::getGlobalPoseStateENU() {
    return estimated_position_;
}

