#include "localization.hpp"

#include <navigation/navigation.hpp>

#ifndef DEBUG
#define DEBUG 0
#endif
#define debug_print(fmt, ...) do { if (DEBUG) { fprintf(stderr, fmt, __VA_ARGS__); }} while (0)

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
    return estimated_height_.status.enabled_status;
}

bool EaglEyeLocalization::hasPlausibleSlipAngle() const {
    return estimated_slip_angle_.status.enabled_status;
}

bool EaglEyeLocalization::hasPlausiblePosition() const {
    return estimated_position_local_.status.enabled_status;
}

bool EaglEyeLocalization::hasPlausibleInterpolatedPosition() const {
    return estimated_position_predicted_.status.enabled_status;
}

bool EaglEyeLocalization::hasPlausibleHeading() const {
    return estimated_heading_.status.enabled_status;
}

bool EaglEyeLocalization::hasPlausiblePitchAngle() const {
    return estimated_pitch_.status.enabled_status;
}

bool EaglEyeLocalization::hasPlausibleAccelerationScaleFactor() const {
    return acc_x_offset_.status.enabled_status;
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
    enu_velocity_.header.stamp = imu_timestamp;

    Position PR = estimated_position_local_;
    PR.header.stamp = imu_timestamp;

    Vector3Stamped EV = enu_velocity_;
    vehicle_kinematics_.header.stamp = imu_timestamp;
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
        &EV,
        &PR,
        &vehicle_kinematics_,
        &vehicle_kinematics_with_cov_
    );
    // If the heading interpolation signal is valid, incorporate our position and velocity prediction
//    if (estimated_heading_interpolated_.status.enabled_status) {
        debug_print("trajectory3d_estimate | Position delta x[%5.14f] y[%5.14f] z[%5.14f], Position enabled [%s] | Position estimate status [%s] | Interpol heading [%s]\n",
            PR.enu_pos.x - estimated_position_local_.enu_pos.x,
            PR.enu_pos.y - estimated_position_local_.enu_pos.y,
            PR.enu_pos.z - estimated_position_local_.enu_pos.z,
            PR.status.enabled_status ? "Y" : "N",
            PR.status.estimate_status ? "Y" : "N",
            estimated_heading_interpolated_.status.enabled_status ? "Y" : "N"
        );
        estimated_position_local_.enu_pos = PR.enu_pos;
        estimated_position_local_.status.enabled_status = PR.status.enabled_status;
        estimated_position_local_.status.estimate_status = PR.status.estimate_status;
        enu_velocity_.vector = EV.vector;
//    } else {
//        debug_print("Trajectory estimator input heading status was invalid [%d]. Dropping estimated position and ENU velocity\n", estimated_heading_interpolated_.status.enabled_status);
//    }
    last_imu_data_ = I;
    has_new_imu_data_ = true;
}

void EaglEyeLocalization::addGnssPvt(const PositionVelocityTimeSolution& PVT) {
    if (PVT.timestamp_ns == last_pvt_.timestamp_ns) {
        debug_print("Error: PositionVelocityTimeSolution was fed twice with the same timestamp. Ignoring., %d\n", 0);
        return;
    }
    GNSSPVT P;
    P.timestamp_ns = PVT.timestamp_ns;
    P.position = PVT.position;
    P.track = PVT.truenorth_heading_deg;
    P.speed = PVT.ground_speed_ms;
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
    enu_velocity_ = EV;

    position_estimate(  // Updates estimated position
        G,
        vehicle_kinematics_,
        velocity_enable_status_,
        estimated_distance_,
        estimated_heading_,
        enu_velocity_,
        position_parameter_,
        &position_status_,
        &estimated_position_local_
    );
    // estimated_position_local_.header.stamp = EV.header.stamp;
    estimated_position_local_.header.stamp = enu_velocity_.header.stamp;
    last_gnss_ = G;
    has_new_gnss_data_ = true;
    debug_print("position status [%s]\n", estimated_position_local_.status.enabled_status ? "Y" : "N");
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
    debug_print("Computing state...%s", "\n");
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
            debug_print("YawRateOffsetStop has NaN or infinite value%s", "\n");
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
        // The below metadata copying is from position_interpolate_node.cpp
        estimated_position_predicted_.header = enu_velocity_.header;
        debug_print("    Feeding position_interpolate_estimate an ENU velocity with timestamp [%ld.%ld]\n",
            enu_velocity_.header.stamp.tv_sec,
            enu_velocity_.header.stamp.tv_nsec
        );
        position_interpolate_estimate(
            estimated_position_local_,
            enu_velocity_,
            estimated_position_local_,   // should be Smoothed Position instead, only for height
            estimated_height_,
            estimated_heading_,
            position_interpolate_parameter_,
            &position_interpolate_status_,
            &estimated_position_predicted_,
            &estimated_llh_
        );
        debug_print("    position_interpolate mode [%s] enabled [%s]\n", has_new_gnss_data_ ? "GNSS" : "DEADRECKONING", estimated_position_predicted_.status.enabled_status ? "Y" : "N");
        debug_print("    position_interpolate local [%4.9f][%4.9f][%4.9f]   predicted [%4.9f][%4.9f][%4.9f]\n",
            estimated_position_local_.enu_pos.x,
            estimated_position_local_.enu_pos.y,
            estimated_position_local_.enu_pos.z,
            estimated_position_predicted_.enu_pos.x,
            estimated_position_predicted_.enu_pos.y,
            estimated_position_predicted_.enu_pos.z
        );
        has_new_gnss_data_ = false;
    } else {
        debug_print("    position interpolation not yet ready: no plausible position%s", "\n");
    }
    debug_print("Done computing state.%s", "\n\n");
}

void EaglEyeLocalization::resetRelativeMotionTrackingOrigin(const GNSSPosition& p) {
    estimated_position_local_.enu_pos.x = 0.0;
    estimated_position_local_.enu_pos.y = 0.0;
    estimated_position_local_.enu_pos.z = 0.0;
    llh2xyz(reinterpret_cast<double*>(const_cast<GNSSPosition*>(&p)), reinterpret_cast<double*>(&estimated_position_local_.ecef_base_pos));
}

GNSSState EaglEyeLocalization::getGlobalPoseStateLLA() {
    GNSSState g;
    if (estimated_position_predicted_.status.enabled_status) {
        g.timestamp_ns = estimated_position_predicted_.header.stamp.tv_sec * 1e9 + estimated_position_predicted_.header.stamp.tv_nsec;
        g.position.lat = estimated_llh_.lat;
        g.position.lon = estimated_llh_.lon;
        g.position.alt_msl = estimated_llh_.alt_msl;
        g.position.surface_stddev_m = estimated_position_predicted_.covariance[4];
        g.position.alt_stddev_m = estimated_position_predicted_.covariance[8];
    }
    return g;
}

GNSSState EaglEyeLocalization::getGlobalPoseStateLLAraw() {
    GNSSState g;
    if (estimated_position_local_.status.enabled_status) {
        g.timestamp_ns = estimated_position_local_.header.stamp.tv_sec * 1e9 + estimated_position_local_.header.stamp.tv_nsec;
        g.position.lat = estimated_llh_.lat;
        g.position.lon = estimated_llh_.lon;
        g.position.alt_msl = estimated_llh_.alt_msl;
        g.position.surface_stddev_m = estimated_position_local_.covariance[4];
        g.position.alt_stddev_m = estimated_position_local_.covariance[8];
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
    return estimated_position_predicted_.enu_pos;
}

Vector3d EaglEyeLocalization::getRelativePositionOriginECEF() {
    return estimated_position_predicted_.ecef_base_pos;
}
