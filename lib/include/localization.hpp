#ifndef EAGLEYE_LIB_HPP_
#define EAGLEYE_LIB_HPP_

#include <navigation/navigation.hpp>

struct PositionVelocityTimeSolution {
    uint64_t timestamp_ns;
    uint32_t gps_time_of_week_ms {  };
    uint8_t num_solution_satellites {  };
    GNSSPosition position {  };
    Vector3d velocity_ned_ms {  };
    float ground_speed_ms {  };
    float truenorth_heading_deg {  };
    float heading_accuracy_estimate_deg {  };
    float speed_accuracy_estimate_ms {  };
};

struct EaglEyeParameters {
    enum class DriveType {
        FWD,
        RWD,
        AWD,
        Invalid
    };
    DriveType drive_type{DriveType::Invalid};
    float wheel_base_m{2.3F};  // This is important to update to an accurate measurement
    float steer_ratio{0.0165};  // How much wheel turn is required to turn the wheel one angle unit (e.g. degree, radian, ...)
    float imu_rate_hz{50.F};  // Output rate of the IMU
    float gnss_rate_hz{1.F};  // Output rate of the GNSS positioning and velocity estimator source
    float vehicle_stop_judgement_threshold_speed_mps{0.021};  // 0.02m/s
    float vehicle_moving_judgement_threshold_speed_mps{2.78F}; // 2.78 m/s == 10 km/h
    float vehicle_slow_judgement_threshold_speed_mps{0.278F}; // 0.278 m/s == 1 km/h

    float heading_data_buffering_init_time_min_s{10.F};  // Minimum seconds to collect data before providing a heading estimate
    float heading_data_buffering_init_time_max_s{30.F};  // ??
    float heading_minimum_gnss_rate{0.25F};  // The ratio of GNSS measurements per estimated trajectory
    float heading_outlier_threshold{0.0524F};
    float heading_outlier_threshold_ratio{0.5F};
    float heading_curve_judgement_threshold_rps{0.0524F}; // Radians per second
    float heading_init_stddev{0.0035F};  // Radians

    float heading_interpolation_sync_search_period_s{2.F};  // ??
    float heading_interpolation_process_noise{0.0005F};  // ??

    float position_data_buffering_interval_m{300.F};
    float position_update_minimum_distance_m{0.1F};
    float position_gnss_reception_threshold{0.1F}; // Mysterious value 0.0-1.0 "Threshold of minimum GNSS reception rate"
    float position_outlier_threshold_m{3.0F}; // meters
    float position_outlier_threshold_ratio{0.5F}; // 0.0 - 1.0 ratio of allowable outliers within the interval
    float position_moving_average_time_s{1.F};  // seconds?

    float position_interpolation_sync_search_period_s{2.F}; // >?
    float position_interpolation_process_noise{0.05F};  // meters

    float slip_angle_manual_coefficient{0.0F}; // ?

    float roll_filter_process_noise{0.01F}; // ??
    float roll_filter_observation_noise{1.F}; // ??

    float trajectory_curve_judgement_threshold{0.017453F};
    float trajectory_deadlock_threshold{1.F};
    float trajectory_sensor_velocity_noise{0.05F};
    float trajectory_sensor_velocity_scale_noise{0.02F};
    float trajectory_sensor_yaw_bias_noise{0.1F};

    float altitude_buffering_init_time_min_s{200.F};
    float altitude_buffering_init_time_max_s{2000.F};
    float altitude_update_distance{0.1F};  // meters?
    float altitude_outlier_threshold{0.3F};
    float altitude_outlier_ratio_threshold{0.5F};
    float altitude_moving_average_interval{1.F}; // seconds?

    float global_velocity_estimator_stop_judgement_threshold_speed_mps{0.02F};  // Threshold speed (m/s) for considering the vehicle stopped
    float global_velocity_estimator_stop_judgement_interval_s{1.F};  // Threshold speed evaluation interval: 1 second
    float global_velocity_estimator_stop_judgement_vibration_variance_rps2{0.000025F}; // Angular velocity vibration variance for stop judgement, radians / second^2.

    float global_velocity_scale_factor_percent{10.F};  // 100%

    float velocity_scale_factor_buffering_init_time_min_s{20.F};  // seconds
    float velocity_scale_factor_buffering_init_time_max_s{400.F}; // seconds
    float velocity_scale_factor_minimum_gnss_rate{0.25F};  // ?????
    std::string velocity_scale_factor_storage_path;
    bool velocity_scale_factor_storage{false};

    float yaw_rate_offset_stop_buffering_init_time_min_s{4.F};
    float yaw_rate_offset_stop_outlier_threshold{0.002F};

    float yaw_rate_offset_buffering_init_time_min_s{30.F};
    float yaw_rate_offset_minimum_gnss_rate{0.25F}; // ???

    float yaw_rate_offset_outlier_threshold{0.002F};
    float yaw_rate_offset_buffering_init_time_max_1st_s{300.F};
    float yaw_rate_offset_buffering_init_time_max_2nd_s{500.F};
};

struct WheelSpeedMeasurement {
    uint64_t timestamp_ns{};
    float wheelspeed_fr_mps{0.F};
    float wheelspeed_fl_mps{0.F};
    float wheelspeed_rr_mps{0.F};
    float wheelspeed_rl_mps{0.F};
};


class EaglEyeLocalization {

  public:
    explicit EaglEyeLocalization(EaglEyeParameters p);

    void addImuMeasurement(const ImuState& I);

    void addGnssPvt(const PositionVelocityTimeSolution& G);

    void addWheelSpeeds(const WheelSpeedMeasurement& ws);

    void addSteeringAngleMeasurement(const double steer_angle);

    void computeState();

    void resetRelativeMotionTrackingOrigin(const GNSSPosition& p);

    bool hasPlausibleHeading() const;
    bool hasPlausiblePosition() const;
    bool vehicleHasMovedOnce() const;
    bool hasPlausibleHeight() const;
    bool hasPlausibleSlipAngle() const;
    bool hasPlausibleInterpolatedPosition() const;
    bool hasPlausiblePitchAngle() const;
    bool hasPlausibleAccelerationScaleFactor() const;

    GNSSState getGlobalPoseStateLLA();

    GNSSState getGlobalPoseStateLLAraw();

    Position getGlobalPoseStateENU();

    Vector3d getRelativePositionOriginECEF();

    Vector3d getRelativePositionENU();

    Vector3d getAttitude();

  private:
    EaglEyeParameters config_{};

    bool has_new_imu_data_{false};
    bool has_new_wheelspeed_data_{false};
    bool has_new_gnss_data_{false};
    bool vehicle_has_moved_{false};

    Heading estimated_heading_{};
    HeadingStatus estimated_heading_status_{};
    HeadingParameter heading_parameter_{};

    Heading estimated_heading_interpolated_{};
    HeadingInterpolateParameter heading_interpolate_parameter_{};
    HeadingInterpolateStatus heading_interpolate_status_{};

    Position estimated_position_local_{};
    PositionParameter position_parameter_{};
    PositionStatus position_status_{};
    PositionStatus estimated_position_status_{};

    Position estimated_position_predicted_{};
    PositionInterpolateParameter position_interpolate_parameter_{};
    PositionInterpolateStatus position_interpolate_status_{};
    GNSSPosition estimated_llh_{};

    Vector3Stamped enu_velocity_{};

    HeightParameter height_parameter_{};
    HeightStatus height_status_{};
    Height estimated_height_{};

    StatusStamped velocity_enable_status_{};

    Distance estimated_distance_{};
    DistanceStatus distance_status_{};

    TrajectoryStatus trajectory_status_{};
    TrajectoryParameter trajectory_parameter_{};

    TwistStamped vehicle_kinematics_{};
    TwistWithCovarianceStamped vehicle_kinematics_with_cov_{};
    GNSSState last_gnss_{};
    GNSSPVT last_pvt_{};
    ImuState last_imu_data_{};

    YawRateOffset yaw_rate_offset_2nd_{};
    YawrateOffsetStatus yaw_rate_offset_status_{};
    YawrateOffsetParameter yaw_rate_offset_parameter_{};

    YawrateOffsetStopParameter yaw_rate_offset_stop_parameter_{};

    YawRateOffset yaw_rate_offset_stop_{};
    double previous_yaw_rate_offset_stop_{};
    YawrateOffsetStopStatus yaw_rate_offset_stop_status_{};

    AngularVelocityOffset angular_velocity_offset_stop_{};

    AccXOffset acc_x_offset_{};
    AccXScaleFactor acc_x_scale_factor_{};

    SlipAngle estimated_slip_angle_{};
    SlipAngle last_slip_angle_{};
    SlipCoefficientParameter slip_coefficient_parameter_{};

    Rolling estimated_roll_{};
    RollingStatus rolling_status_{};
    RollingParameter rolling_parameter_{};

    Pitching estimated_pitch_{};

    SlipangleParameter slip_angle_parameter_{};

    VelocityScaleFactor velocity_scale_factor_{};
    double previous_velocity_scale_factor_{};
    VelocityScaleFactorParameter velocity_scale_factor_parameter_{};
    VelocityScaleFactor default_velocity_scale_factor_{};
    VelocityScaleFactorStatus velocity_scale_factor_status_{};

};

#endif  // EAGLEYE_LIB_HPP_

