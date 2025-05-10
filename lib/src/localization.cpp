#include "localization.hpp"

#include <navigation/navigation.hpp>

EaglEyeLocalization::EaglEyeLocalization(EaglEyeParameters p) {
    velocity_enable_status_.status.enabled_status = true; // HACK
    estimated_distance_.distance = 0.0;

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
    position_parameter_.update_distance = p.position_update_minimum_distance_m;
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

    height_parameter_.imu_rate = p.imu_rate_hz;
    height_parameter_.gnss_rate = p.gnss_rate_hz;
    height_parameter_.moving_judgement_threshold = p.vehicle_stop_judgement_threshold_speed_mps;
    height_parameter_.estimated_minimum_interval = p.altitude_buffering_init_time_min_s;
    height_parameter_.estimated_maximum_interval = p.altitude_buffering_init_time_max_s;
    height_parameter_.update_distance = p.altitude_update_distance;
    height_parameter_.gnss_receiving_threshold = 0.1; // HACK FIXME
    height_parameter_.outlier_threshold = p.altitude_outlier_threshold;
    height_parameter_.outlier_ratio_threshold = p.altitude_outlier_ratio_threshold;
    height_parameter_.moving_average_time = p.altitude_moving_average_interval;
}

bool EaglEyeLocalization::vehicleHasMovedOnce() const {
    return vehicle_has_moved_;
}

bool EaglEyeLocalization::hasPlausibleHeight() const {
    if (estimated_height_.status.enabled_status) {
        return true;
    } else {
        return false;
    }
}

bool EaglEyeLocalization::hasPlausibleSlipAngle() const {
    if (estimated_slip_angle_.status.enabled_status) {
        return true;
    } else {
        return false;
    }
}

bool EaglEyeLocalization::hasPlausiblePosition() const {
    if (estimated_position_.status.enabled_status) {
        return true;
    } else {
        return false;
    }
}

bool EaglEyeLocalization::hasPlausibleInterpolatedPosition() const {
    if (estimated_position_final_.status.enabled_status) {
        return true;
    } else {
        return false;
    }
}

bool EaglEyeLocalization::hasPlausibleHeading() const {
    if (estimated_heading_.status.enabled_status) {
        return true;
    } else {
        return false;
    }
}

void EaglEyeLocalization::addImuMeasurement(const ImuState& imu) {
    ImuState I = imu;
    timespec imu_timestamp{};
    imu_timestamp.tv_sec = I.timestamp_ns / 1e9;
    imu_timestamp.tv_nsec = I.timestamp_ns - imu_timestamp.tv_sec * 1e9;

    // Apply forward acelleration corrections if available
    if (acc_x_offset_.status.enabled_status && acc_x_scale_factor_.status.enabled_status) {
        I.linear_acceleration_mpss.x = I.linear_acceleration_mpss.x * acc_x_scale_factor_.acc_x_scale_factor + acc_x_offset_.acc_x_offset;
    }
    // Some kind of coordinate system conversion. Reason unknown.
    I.angular_velocity_rps.z = -1.F * (I.angular_velocity_rps.z + angular_velocity_offset_stop_.angular_velocity_offset.z);


    // Prepare slip angle estimation
    estimated_slip_angle_.header.stamp = imu_timestamp;
    slip_angle_estimate(  // Estimate the wheel slip angle (actual motion angle relative to wheel angle)
        I,
        vehicle_kinematics_,
        velocity_enable_status_,
        yaw_rate_offset_stop_,
        yaw_rate_offset_2nd_,
        slip_angle_parameter_,
        &estimated_slip_angle_
    );
    std::printf("Slip angle [%s]\n", estimated_slip_angle_.status.enabled_status ? "Y" : "N");
    estimated_pitch_.header.stamp = imu_timestamp;
    pitching_estimate(  // Estimate the pitch
        I,
        last_gnss_,
        vehicle_kinematics_,
        estimated_distance_,
        height_parameter_,
        &height_status_,
        &estimated_height_,
        &estimated_pitch_,
        &acc_x_offset_,
        &acc_x_scale_factor_
    );
    std::printf("Height enabled [%s]\n", estimated_height_.status.enabled_status ? "Y" : "N");
   std::printf("Pitch enabled [%s]\n", estimated_pitch_.status.enabled_status ? "Y" : "N");
    std::printf("ACC X Scale factor enabled [%s]\n", acc_x_offset_.status.enabled_status ? "Y" : "N");
    rolling_estimate(  // Estimate the roll
        I,
        vehicle_kinematics_,
        yaw_rate_offset_stop_,
        yaw_rate_offset_2nd_,
        rolling_parameter_,
        &rolling_status_,
        &estimated_roll_
    );
    estimated_roll_.header.stamp = estimated_slip_angle_.header.stamp;
    std::printf("Roll enabled [%s]\n", estimated_roll_.status.enabled_status ? "Y" : "N");
    heading_interpolate_estimate(
        I,
        vehicle_kinematics_,
        yaw_rate_offset_stop_,
        yaw_rate_offset_2nd_,
        estimated_heading_,
        estimated_slip_angle_,
        heading_interpolate_parameter_,
        &heading_interpolate_status_,
        &estimated_heading_interpolated_
    );
    Position PR;  // throwaway, relative estimation only
    trajectory3d_estimate(
        I,
        vehicle_kinematics_,
        velocity_enable_status_,
        estimated_heading_interpolated_,
        yaw_rate_offset_stop_,
        yaw_rate_offset_2nd_,
        estimated_pitch_,
        trajectory_parameter_,
        &trajectory_status_,
        &enu_velocities_,
        &PR,
        &vehicle_kinematics_,
        &vehicle_kinematics_with_cov_
    );
    std::printf("Trajectory estimator estimated (discarded) relative motion [%3.9f|%3.9f|%3.9f]\n", PR.enu_pos.x, PR.enu_pos.y, PR.enu_pos.y);
    last_imu_data_ = I;
    has_new_imu_data_ = true;
}

void EaglEyeLocalization::addGnssPvt(const PositionVelocityTimeSolution& PVT) {
    GNSSPVT P;
    P.timestamp_ns = PVT.timestamp_ns;
    P.position = PVT.position;
    P.track = PVT.truenorth_heading_deg;
    P.speed = PVT.ground_speed_ms;
    if (PVT.timestamp_ns == last_pvt_.timestamp_ns) {
        std::fprintf(stderr, "Error: PositionVelocityTimeSolution was fed twice with the same timestamp. Ignoring.\n");
        return;
    }
    heading_estimate(
        P,
        last_imu_data_,
        vehicle_kinematics_,
        yaw_rate_offset_stop_,
        yaw_rate_offset_2nd_,
        estimated_slip_angle_,
        estimated_heading_,
        heading_parameter_,
        &estimated_heading_status_,
        &estimated_heading_
    );
    last_pvt_ = P;
    GNSSState G;
    G.timestamp_ns = P.timestamp_ns;
    G.position = P.position;
    Vector3Stamped EV;
    EV.header.stamp.tv_sec = P.timestamp_ns / 1e9;
    EV.header.stamp.tv_nsec = P.timestamp_ns - EV.header.stamp.tv_sec * 1e9;
    // Convert froM NED to ENU
    EV.vector.x = PVT.velocity_ned_ms.y;
    EV.vector.y = PVT.velocity_ned_ms.x;
    EV.vector.z = - PVT.velocity_ned_ms.z;  // Note the flip
    enu_velocities_ = EV;
    position_estimate(  // Updates estimated position
        G,
        vehicle_kinematics_,
        velocity_enable_status_,
        estimated_distance_,
        estimated_heading_,
        EV,
        position_parameter_,
        &position_status_,
        &estimated_position_
    );
    estimated_position_.header = EV.header;
    last_gnss_ = G;
    std::printf("position status [%s]\n", estimated_position_.status.enabled_status ? "Y" : "N");
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
    vehicle_kinematics_.header.stamp.tv_sec = ws.timestamp_ns / 1e9;
    vehicle_kinematics_.header.stamp.tv_nsec = ws.timestamp_ns - vehicle_kinematics_.header.stamp.tv_sec * 1e9;
    vehicle_kinematics_.twist.linear.x = driven_wheel_speed_average_mps;
    has_new_wheelspeed_data_ = true;

    if (vehicle_has_moved_ == false && vehicle_kinematics_.twist.linear.x > config_.vehicle_moving_judgement_threshold_speed_mps) {
        vehicle_has_moved_ = true;
    }

    if (vehicle_has_moved_ == false) {
        velocity_scale_factor_ = default_velocity_scale_factor_;
    } else {

        TwistStamped corrected_velocity{};
        velocity_scale_factor_estimate(
            last_pvt_,
            vehicle_kinematics_,
            velocity_scale_factor_parameter_,
            &velocity_scale_factor_status_,
            &corrected_velocity,
            &velocity_scale_factor_
        );
        corrected_velocity.header.stamp = vehicle_kinematics_.header.stamp;
        if (std::isfinite(velocity_scale_factor_.scale_factor) == false) {
            corrected_velocity.twist.linear.x  = vehicle_kinematics_.twist.linear.x * previous_velocity_scale_factor_;
            velocity_scale_factor_.scale_factor = previous_velocity_scale_factor_;
            velocity_scale_factor_.status.is_abnormal = true;
            velocity_scale_factor_.status.error_code = Status::NAN_OR_INFINITE;
        } else if (config_.global_velocity_scale_factor_percent / 100 < std::abs(1.0 - velocity_scale_factor_.scale_factor)) {
            corrected_velocity.twist.linear.x = vehicle_kinematics_.twist.linear.x * previous_velocity_scale_factor_;
            velocity_scale_factor_.scale_factor = previous_velocity_scale_factor_;
            velocity_scale_factor_.status.is_abnormal = true;
            velocity_scale_factor_.status.error_code = Status::TOO_LARGE_OR_SMALL;
        } else {
            previous_velocity_scale_factor_ = velocity_scale_factor_.scale_factor;
        }
        vehicle_kinematics_ = corrected_velocity;
        velocity_enable_status_.header = velocity_scale_factor_.header;
        velocity_enable_status_.status = velocity_scale_factor_.status;
    }
}

void EaglEyeLocalization::addSteeringAngleMeasurement(const double steer_angle_rad) {
    const float R = config_.wheel_base_m / std::tan(steer_angle_rad * config_.steer_ratio);
    const double W = vehicle_kinematics_.twist.linear.x / R;
    vehicle_kinematics_.twist.angular.z = W;
}

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
    if (has_new_imu_data_) {
        has_new_imu_data_ = false;

        yaw_rate_offset_stop_estimate(
            vehicle_kinematics_,
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
    }
    if (has_new_wheelspeed_data_) {
        has_new_wheelspeed_data_ = false;
        distance_estimate(
            vehicle_kinematics_,
            &distance_status_,
            &estimated_distance_
        );
    }
    if (hasPlausiblePosition()) {
        position_interpolate_estimate(
            estimated_position_,
            enu_velocities_,
            estimated_position_,   // should be Smoothed Position instead, only for height
            estimated_height_,
            estimated_heading_,
            position_interpolate_parameter_,
            &position_interpolate_status_,
            &estimated_position_final_,
            &estimated_llh_
        );
        std::printf("position_interpolate enabled [%s]\n", estimated_position_final_.status.enabled_status ? "Y" : "N");
    }

}

void EaglEyeLocalization::resetRelativeMotionTrackingOrigin(const GNSSPosition& p) {
    estimated_position_.enu_pos.x = 0.0;
    estimated_position_.enu_pos.y = 0.0;
    estimated_position_.enu_pos.z = 0.0;
    llh2xyz(reinterpret_cast<double*>(const_cast<GNSSPosition*>(&p)), reinterpret_cast<double*>(&estimated_position_.ecef_base_pos));
}

GNSSState EaglEyeLocalization::getGlobalPoseStateLLA() {
    GNSSState g;
    if (estimated_position_final_.status.enabled_status) {
        g.timestamp_ns = estimated_position_final_.header.stamp.tv_sec * 1e9 + estimated_position_final_.header.stamp.tv_nsec;
        g.position.lat = estimated_llh_.lat;
        g.position.lon = estimated_llh_.lon;
        g.position.alt_msl = estimated_llh_.alt_msl;
        g.position.surface_stddev_m = estimated_position_final_.covariance[4];
        g.position.alt_stddev_m = estimated_position_final_.covariance[8];
    }
    return g;
}

GNSSState EaglEyeLocalization::getGlobalPoseStateLLAraw() {
    GNSSState g;
    if (estimated_position_.status.enabled_status) {
        g.timestamp_ns = estimated_position_.header.stamp.tv_sec * 1e9 + estimated_position_.header.stamp.tv_nsec;
        g.position.lat = estimated_llh_.lat;
        g.position.lon = estimated_llh_.lon;
        g.position.alt_msl = estimated_llh_.alt_msl;
        g.position.surface_stddev_m = estimated_position_.covariance[4];
        g.position.alt_stddev_m = estimated_position_.covariance[8];
    }
    return g;
}

Vector3d EaglEyeLocalization::getAttitude() {
    Vector3d A;
    A.x = estimated_roll_.rolling_angle;
    A.y = estimated_pitch_.pitching_angle;
    A.z = estimated_heading_.heading_angle;
    return A;
}

Vector3d EaglEyeLocalization::getRelativePositionENU() {
    return estimated_position_final_.enu_pos;
}

Vector3d EaglEyeLocalization::getRelativePositionOriginECEF() {
    return estimated_position_final_.ecef_base_pos;
}
