// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef NAVIGATION_H
#define NAVIGATION_H


#include <cstdint>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include <boost/circular_buffer.hpp>
#include <cmath>
#include <numeric>

#include <coordinate/coordinate.hpp>

struct Status {
    static constexpr uint8_t NAN_OR_INFINITE{0};
    static constexpr uint8_t TOO_LARGE_OR_SMALL{0};
    bool enabled_status;
    bool estimate_status;
    bool is_abnormal;
    uint8_t error_code;
};

struct Vector3d {
    double x {  };
    double y {  };
    double z {  };
};

struct GNSSPosition {
    double lat {  };
    double lon {  };
    double alt_msl {  };
    float surface_stddev_m {  };
    float alt_stddev_m {  };
};

struct GNSSState {
    uint64_t timestamp_ns {  };
    GNSSPosition position {  };
};

struct GNSSPVT {
    uint64_t timestamp_ns{};
    float track{0.F};   // Heading in degrees, true north
    float speed{0.F};
    GNSSPosition position{};
};

struct ImuState {
    uint64_t timestamp_ns {  };
    Quaternion attitude {  };
    double attitude_covariance[9];

    Vector3d angular_velocity_rps {  };
    Vector3d linear_acceleration_mpss {  };

    double angular_velocity_covariance[9];
    double linear_acceleration_covariance[9];
};

struct Distance {
    double distance{0.0};
    Status status;
};

struct Header {
    uint32_t seq;
    timespec stamp;
    std::string frame_id;
};

struct Height {
    double height{0.0};
    Status status;
};

struct Twist {
    Vector3d linear;
    Vector3d angular;
};

struct TwistWithCovariance {
    Twist twist;
    double covariance[36];
};

struct TwistWithCovarianceStamped {
    Header header;
    TwistWithCovariance twist;
};

struct Vector3Stamped {
    Vector3d vector;
    Header header;
};

struct StatusStamped {
    Header header;
    Status status;
};

struct TwistStamped {
    Header header;
    Twist twist;
};

struct Pose {
    Vector3d position;
    Quaternion orientation;
};

struct PoseStamped {
    Header header;
    Pose pose;
};

struct YawRateOffset {
    double yaw_rate_offset;
    Status status;
};

struct VelocityScaleFactor {
    Header header;
    double scale_factor;
    Status status;
};

struct AccXScaleFactor {
    double acc_x_scale_factor;
    Status status;
};

struct Rolling {
    Header header;
    double rolling_angle;
    Status status;
};

struct Pitching {
    Header header;
    double pitching_angle;
    Status status;
};

struct Heading {
    Header header;
    double heading_angle;
    double variance;
    Status status;
};

struct AccYOffset {
    double acc_y_offset;
    Status status;
};

struct AccXOffset {
    double acc_x_offset;
    Status status;
};

struct SlipAngle {
    Header header;
    double coefficient;
    double slip_angle;
    Status status;
};

struct Position {
    Header header;
    Vector3d enu_pos;
    Vector3d ecef_base_pos;
    double covariance[9];
    Status status;
};

struct AngularVelocityOffset {
    Vector3d angular_velocity_offset;
    Status status;
};

struct VelocityScaleFactorParameter
{
  double imu_rate{25};
  double gnss_rate{2};
  double moving_judgement_threshold{2.78};
  double estimated_minimum_interval{20};
  double estimated_maximum_interval{400};
  double gnss_receiving_threshold{0.25};
  bool save_velocity_scale_factor{false};
};

struct VelocityScaleFactorStatus
{
  std::vector<bool> gnss_status_buffer;
  std::vector<double> doppler_velocity_buffer;
  std::vector<double> velocity_buffer;
  int tow_last, estimated_number;
  double rmc_time_last;
  double velocity_scale_factor_last;
  bool estimate_start_status;
};

struct DistanceStatus
{
  bool is_first_data{true};
  double time_last;
};

struct YawrateOffsetStopParameter
{
  double imu_rate{25};
  double estimated_interval{4};
  double stop_judgement_threshold{0.01};
  double outlier_threshold{0.002};
};

struct YawrateOffsetStopStatus
{
  int stop_count;
  double yaw_rate_offset_stop_last;
  bool estimate_start_status;
  std::vector<double> yaw_rate_buffer;
};

struct YawrateOffsetParameter
{
  double imu_rate{25};
  double gnss_rate{2};
  double moving_judgement_threshold{2.78};
  double estimated_minimum_interval{30};
  double estimated_maximum_interval{300};
  double gnss_receiving_threshold{0.25};
  double outlier_threshold{0.002};
};

struct YawrateOffsetStatus
{
  bool estimate_start_status;
  int estimated_preparation_conditions;
  int heading_estimate_status_count;
  int estimated_number;
  std::vector<double> time_buffer;
  std::vector<double> yaw_rate_buffer;
  std::vector<double> heading_angle_buffer;
  std::vector<double> correction_velocity_buffer;
  std::vector<bool> heading_estimate_status_buffer;
  std::vector<double> yaw_rate_offset_stop_buffer;
};

struct HeadingParameter
{
  double imu_rate{25};
  double gnss_rate{2};
  double stop_judgement_threshold{0.01};
  double moving_judgement_threshold{2.78};
  double estimated_minimum_interval{10};
  double estimated_maximum_interval{30};
  double gnss_receiving_threshold{0.25};
  double outlier_threshold{0.0524};
  double outlier_ratio_threshold{0.5};
  double curve_judgement_threshold{0.0873};
  double init_STD{0.0035};
};

struct HeadingStatus
{
  int tow_last;
  double rmc_time_last;
  double ros_time_last;
  int estimated_number;
  std::vector<double> time_buffer;
  std::vector<double> heading_angle_buffer;
  std::vector<double> yaw_rate_buffer;
  std::vector<double> correction_velocity_buffer;
  std::vector<double> yaw_rate_offset_stop_buffer;
  std::vector<double> yaw_rate_offset_buffer;
  std::vector<double> slip_angle_buffer;
  std::vector<double> gnss_status_buffer;
};

struct RtkHeadingParameter
{
  double imu_rate{25};
  double gnss_rate{2};
  double stop_judgement_threshold;
  double slow_judgement_threshold{0.278};
  double update_distance;
  double estimated_minimum_interval;
  double estimated_maximum_interval;
  double gnss_receiving_threshold;
  double outlier_threshold;
  double outlier_ratio_threshold;
  double curve_judgement_threshold;
};

struct RtkHeadingStatus
{
  int tow_last;
  int estimated_number;
  double last_rtk_heading_angle;
  std::vector<double> time_buffer;
  std::vector<double> heading_angle_buffer;
  std::vector<double> yaw_rate_buffer;
  std::vector<double> correction_velocity_buffer;
  std::vector<double> yaw_rate_offset_stop_buffer;
  std::vector<double> yaw_rate_offset_buffer;
  std::vector<double> slip_angle_buffer;
  std::vector<double> gnss_status_buffer;
  std::vector<double> distance_buffer;
  std::vector<double> latitude_buffer;
  std::vector<double> longitude_buffer;
  std::vector<double> altitude_buffer;
  std::vector<int> gga_status_buffer;
};

struct HeadingInterpolateParameter
{
  double imu_rate{25};
  double stop_judgement_threshold{0.01};
  double sync_search_period;
  double proc_noise;
};

struct HeadingInterpolateStatus
{
  int number_buffer;
  int heading_estimate_status_count;
  bool heading_estimate_start_status;
  double heading_stamp_last;
  double time_last;
  double provisional_heading_angle;
  std::vector<double> provisional_heading_angle_buffer;
  std::vector<double> imu_stamp_buffer;
  double heading_variance_last;
};

struct PositionParameter
{
  double ecef_base_pos_x;
  double ecef_base_pos_y;
  double ecef_base_pos_z;
  double tf_gnss_translation_x;
  double tf_gnss_translation_y;
  double tf_gnss_translation_z;
  double tf_gnss_rotation_x;
  double tf_gnss_rotation_y;
  double tf_gnss_rotation_z;
  double tf_gnss_rotation_w;
  std::string tf_gnss_parent_frame;
  std::string tf_gnss_child_frame;

  double imu_rate{25};
  double gnss_rate{2};
  double moving_judgement_threshold{2.78};
  double estimated_interval{300};
  double update_distance{0.1};
  double gnss_receiving_threshold{0.25};
  double outlier_threshold{3.0};
  double outlier_ratio_threshold{0.5};
  double gnss_error_covariance{0.5};
};

struct PositionStatus
{
  int estimated_number;
  int tow_last;
  int heading_estimate_status_count;
  double nmea_time_last;
  double time_last;
  double enu_relative_pos_x, enu_relative_pos_y, enu_relative_pos_z;
  double distance_last;
  double enu_pos[3];
  bool gnss_update_failure;
  std::vector<double> enu_pos_x_buffer, enu_pos_y_buffer,  enu_pos_z_buffer;
  std::vector<double> enu_relative_pos_x_buffer, enu_relative_pos_y_buffer, enu_relative_pos_z_buffer;
  std::vector<double> correction_velocity_buffer;
  std::vector<double> distance_buffer;
};

struct PositionInterpolateParameter
{
  double imu_rate{25};
  double stop_judgement_threshold;
  double sync_search_period{2};
  double proc_noise{0.0005};
};

struct PositionInterpolateStatus
{
  int position_estimate_status_count;
  int number_buffer;
  bool position_estimate_start_status;
  bool is_estimate_start{false};
  double position_stamp_last;
  double time_last;
  double provisional_enu_pos_x;
  double provisional_enu_pos_y;
  double provisional_enu_pos_z;
  std::vector<double> provisional_enu_pos_x_buffer;
  std::vector<double> provisional_enu_pos_y_buffer;
  std::vector<double> provisional_enu_pos_z_buffer;
  std::vector<double> imu_stamp_buffer;
  Eigen::MatrixXd position_covariance_last;
};

struct SlipangleParameter
{
  double manual_coefficient{0.0};
  double stop_judgement_threshold{0.01};
};

struct SlipCoefficientParameter
{
  double imu_rate{25};
  double estimated_minimum_interval{2};
  double estimated_maximum_interval{100};
  double stop_judgement_threshold{0.01};
  double moving_judgement_threshold{2.78};
  double curve_judgement_threshold{0.017453};
  double lever_arm{0.0};
};

struct SlipCoefficientStatus
{
  double heading_estimate_status_count;
  std::vector<double> doppler_slip_buffer;
  std::vector<double> acceleration_y_buffer;
};

struct SmoothingParameter
{
  double gnss_rate{2};
  double moving_judgement_threshold{2.78};
  double moving_average_time;
  double moving_ratio_threshold;
  double ecef_base_pos_x;
  double ecef_base_pos_y;
  double ecef_base_pos_z;
};

struct SmoothingStatus
{
  int estimated_number;
  double last_pos[3];
  std::vector<double> time_buffer;
  std::vector<double> enu_pos_x_buffer, enu_pos_y_buffer,  enu_pos_z_buffer;
  std::vector<double> correction_velocity_buffer;
};

struct TrajectoryParameter
{
  double stop_judgement_threshold{0.01};
  double curve_judgement_threshold;
  double sensor_noise_velocity;
  double sensor_scale_noise_velocity;
  double sensor_noise_yaw_rate;
  double sensor_bias_noise_yaw_rate;
};

struct TrajectoryStatus
{
  int estimate_status_count;
  double heading_last;
  double time_last;
};

struct HeightParameter
{
  double imu_rate{50};
  double gnss_rate{4};
  double moving_judgement_threshold{2.78};
  double estimated_minimum_interval{200};
  double estimated_maximum_interval{2000};
  double update_distance{1.0};
  double gnss_receiving_threshold{0.1};
  double outlier_threshold{0.3};
  double outlier_ratio_threshold{0.5};
  double moving_average_time{1.0};
};

struct HeightStatus
{
  double relative_height_G;
  double relative_height_diffvel;
  double relative_height_offset;
  double acceleration_offset_linear_x_last;
  double acceleration_SF_linear_x_last;
  double height_last;
  double time_last;
  double distance_last;
  double correction_velocity_x_last;
  double gga_time_last;
  double pitching_angle_last;
  bool height_estimate_start_status;
  bool estimate_start_status;
  bool acceleration_SF_estimate_status;
  int data_number;
  bool flag_reliability;
  std::vector<double> height_buffer;
  std::vector<double> height_buffer2;
  std::vector<double> relative_height_G_buffer;
  std::vector<double> relative_height_diffvel_buffer;
  std::vector<double> relative_height_offset_buffer;
  std::vector<double> correction_relative_height_buffer;
  std::vector<double> correction_relative_height_buffer2;
  std::vector<double> correction_velocity_buffer;
  std::vector<double> distance_buffer;
  std::vector<double> acc_buffer;
};

struct RtkDeadreckoningParameter
{
  double stop_judgement_threshold;
  double ecef_base_pos_x;
  double ecef_base_pos_y;
  double ecef_base_pos_z;
  bool use_ecef_base_position{false};
  double tf_gnss_translation_x;
  double tf_gnss_translation_y;
  double tf_gnss_translation_z;
  double tf_gnss_rotation_x;
  double tf_gnss_rotation_y;
  double tf_gnss_rotation_z;
  double tf_gnss_rotation_w;
  std::string tf_gnss_parent_frame;
  std::string tf_gnss_child_frame;
  double rtk_fix_STD;
  double proc_noise;
};

struct RtkDeadreckoningStatus
{
  int position_estimate_status_count;
  int number_buffer;
  bool position_estimate_start_status;
  bool ecef_base_pos_status;
  double position_stamp_last;
  double time_last;
  double provisional_enu_pos_x;
  double provisional_enu_pos_y;
  double provisional_enu_pos_z;
  std::vector<double> provisional_enu_pos_x_buffer;
  std::vector<double> provisional_enu_pos_y_buffer;
  std::vector<double> provisional_enu_pos_z_buffer;
  std::vector<double> imu_stamp_buffer;
  Eigen::MatrixXd position_covariance_last;
};

struct EnableAdditionalRollingParameter
{
  double imu_rate{25};
  double stop_judgement_threshold;
  double update_distance;
  double moving_average_time;
  double sync_judgement_threshold;
  double sync_search_period;
};

struct EnableAdditionalRollingStatus
{
  double distance_last;
  double acc_offset_sum;
  double yaw_rate;
  double rollrate;
  double imu_acceleration_y;
  double rollrate_offset_stop;
  double imu_time_last;
  double localization_time_last;
  int acc_offset_data_count;
  std::vector<double> roll_rate_interpolate_buffer;
  std::vector<double> rolling_estimated_buffer;
  std::vector<double> imu_time_buffer;
  std::vector<double> yaw_rate_buffer;
  std::vector<double> velocity_buffer;
  std::vector<double> yaw_rate_offset_buffer;
  std::vector<double> acceleration_y_buffer;
  std::vector<double> distance_buffer;
};

struct RollingParameter
{
  double stop_judgement_threshold{0.01};
  double filter_process_noise{0.01};
  double filter_observation_noise{1.0};
};

struct RollingStatus
{
  double acceleration_y_last;
  double acceleration_y_variance_last;
  double rolling_last;
  bool data_status;
};


extern void velocity_scale_factor_estimate( const TwistStamped, const VelocityScaleFactorParameter,
  VelocityScaleFactorStatus*, TwistStamped*, VelocityScaleFactor*);

extern void velocity_scale_factor_estimate(const GNSSPVT, const TwistStamped, const VelocityScaleFactorParameter,
  VelocityScaleFactorStatus*, TwistStamped*, VelocityScaleFactor*);

extern void distance_estimate(const TwistStamped,
  DistanceStatus*, Distance*);

extern void yaw_rate_offset_stop_estimate(const TwistStamped, const ImuState, const YawrateOffsetStopParameter,
  YawrateOffsetStopStatus*, YawRateOffset*);

extern void yaw_rate_offset_estimate(const TwistStamped, const YawRateOffset,const Heading, const ImuState, const YawrateOffsetParameter,
  YawrateOffsetStatus*, YawRateOffset*);

extern void heading_estimate( const ImuState, const TwistStamped, const YawRateOffset, const YawRateOffset,  const SlipAngle, const Heading, const HeadingParameter, 
  HeadingStatus*, Heading*);

extern void heading_estimate(const GNSSPVT, const ImuState, const TwistStamped, const YawRateOffset, const YawRateOffset,  const SlipAngle, const Heading, const HeadingParameter, 
  HeadingStatus*, Heading*);

extern void heading_estimate(const Heading, const ImuState, const TwistStamped, const YawRateOffset, const YawRateOffset,  const SlipAngle, const Heading, const HeadingParameter, 
  HeadingStatus*, Heading*);

extern void position_estimate( const TwistStamped, const StatusStamped, const Distance, const Heading, const Vector3Stamped, const PositionParameter, 
  PositionStatus*, Position*);

extern void position_estimate(const GNSSState gga, const TwistStamped, const StatusStamped, const Distance, const Heading, const Vector3Stamped, const PositionParameter, 
  PositionStatus*, Position*);

extern void slip_angle_estimate(const ImuState,const TwistStamped,const StatusStamped,const YawRateOffset, const YawRateOffset,const SlipangleParameter, 
  SlipAngle*);

extern void slip_coefficient_estimate(const ImuState,const TwistStamped, const YawRateOffset,const YawRateOffset,const Heading,const SlipCoefficientParameter,
  SlipCoefficientStatus*,double*);

extern void smoothing_estimate(const TwistStamped,const SmoothingParameter,
  SmoothingStatus*, Position*);

extern void trajectory_estimate(const ImuState,const TwistStamped,const StatusStamped,const Heading,const YawRateOffset, const YawRateOffset,const TrajectoryParameter,
  TrajectoryStatus*, Vector3Stamped*, Position*, TwistStamped*, TwistWithCovarianceStamped*);

extern void heading_interpolate_estimate(const ImuState,const TwistStamped,const YawRateOffset, const YawRateOffset,const Heading,const SlipAngle,const HeadingInterpolateParameter,
  HeadingInterpolateStatus*, Heading*);

extern void position_interpolate_estimate(const Position, const Vector3Stamped,const Position, const Height,const Heading,const PositionInterpolateParameter,
  PositionInterpolateStatus*, Position*,GNSSPosition*);

extern void pitching_estimate(const ImuState,const GNSSState,const TwistStamped,const Distance, const HeightParameter,
  HeightStatus*, Height*, Pitching*, AccXOffset*, AccXScaleFactor*);

extern void trajectory3d_estimate(const ImuState,const TwistStamped,const StatusStamped,const Heading, const YawRateOffset,const YawRateOffset,const Pitching,const TrajectoryParameter,
  TrajectoryStatus*, Vector3Stamped*, Position*,TwistStamped*, TwistWithCovarianceStamped*);

extern void enable_additional_rolling_estimate(const TwistStamped, const StatusStamped,const YawRateOffset ,const YawRateOffset,
  const Distance,const ImuState,const PoseStamped,const AngularVelocityOffset,
  const EnableAdditionalRollingParameter,EnableAdditionalRollingStatus*,
  Rolling*, AccYOffset*);

extern void rolling_estimate(const ImuState,const TwistStamped,const YawRateOffset, const YawRateOffset,const RollingParameter,
  RollingStatus*, Rolling*);




class VelocityEstimator
{
  public:
    VelocityEstimator();

    // Parameter setting
    void setParam(std::string yaml_file);

    // Main estimate function
    void VelocityEstimate(const ImuState, const GNSSState, TwistStamped*);

    Status getStatus();

  private:

    class PitchrateOffsetStopEstimator
    {
      public:
        PitchrateOffsetStopEstimator();

        void setParam(std::string yaml_file);
        bool PitchrateOffsetStopEstimate(double pitchrate, double stop_status);

        double pitchrate_offset;
        Status pitchrate_offset_status;

      private:
        struct Param
        {
          int imu_rate{25};
          int estimated_interval;
          int buffer_count_max;
        };
        PitchrateOffsetStopEstimator::Param param;

        int stop_count;
        std::vector<double> pitchrate_buffer;
    };

    class PitchingEstimator
    {
      public:
        PitchingEstimator();

        void setParam(std::string yaml_file);
        bool PitchingEstimate(double imu_time_last, double doppler_velocity, double rtkfix_velocity,
                              double pitchrate, double pitchrate_offset, double rtkfix_pitching,
                              bool navsat_update_status, bool stop_status);

        double pitching;
        Status pitching_status;

      private:
        struct Param
        {
          double imu_rate{25};
          double gnss_rate{2};
          double estimated_interval;
          double buffer_max;
          double outlier_threshold;
          double estimated_velocity_threshold;
          double slow_judgement_threshold{0.278};
          double gnss_receiving_threshold;
          double estimated_gnss_coefficient;
          double outlier_ratio_threshold;
          double estimated_coefficient;
        };
        PitchingEstimator::Param param;

        std::vector<double> time_buffer;
        std::vector<double> corrected_pitchrate_buffer;
        std::vector<double> rtkfix_pitching_buffer;
        std::vector<bool> use_gnss_status_buffer;
        std::vector<bool> navsat_update_status_buffer;
    };

    class AccelerationOffsetEstimator
    {
      public:
        AccelerationOffsetEstimator();

        void setParam(std::string yaml_file);
        bool AccelerationOffsetEstimate(double imu_time_last, double rtkfix_velocity, double pitching,
                              double acceleration, bool navsat_update_status);

        double filtered_acceleration;
        double acceleration_offset;
        Status acceleration_offset_status;

      private:
        struct Param
        {
          double imu_rate{25};
          double gnss_rate{2};
          double estimated_minimum_interval;
          double estimated_maximum_interval;
          double buffer_min;
          double buffer_max;
          double filter_process_noise;
          double filter_observation_noise;
        };
        AccelerationOffsetEstimator::Param param;

        double acceleration_last;
        double acceleration_variance_last;
        std::vector<double> time_buffer;
        std::vector<double> pitching_buffer; 
        std::vector<double> filtered_acceleration_buffer;
        std::vector<double> rtkfix_velocity_buffer; 
        std::vector<double> navsat_update_status_buffer;
    };

    // Parameter
    struct Param
    {
      double ecef_base_pos_x;
      double ecef_base_pos_y;
      double ecef_base_pos_z;
      bool use_ecef_base_position{false};

      double imu_rate{25};
      double gnss_rate{2};

      double gga_downsample_time;
      double stop_judgement_interval;
      double stop_judgement_velocity_threshold;
      double stop_judgement_buffer_maxnum;
      double variance_threshold;

      // doppler fusion parameter
      double estimated_interval;
      double buffer_max;
      double gnss_receiving_threshold;
      double estimated_gnss_coefficient;
      double outlier_ratio_threshold;
      double estimated_coefficient;
      double outlier_threshold;
    };
    VelocityEstimator::Param param;

    // imu variables
    double acceleration;
    double pitchrate;
    double imu_time_last;

    // rtklib_nav variables
    double gnss_rate{2};
    double doppler_velocity;
    double rtklib_nav_time_last;
    bool rtklib_update_status;

    // gga variables
    double ecef_base_position[3]; 
    bool ecef_base_position_status;
    double gga_time_last;
    double gga_position_enu_last[3]; 
    int gga_status_last;
    double rtkfix_velocity; 
    double rtkfix_pitching; 
    bool navsat_update_status;

    // stop judgement variables
    bool stop_status;
    std::vector<double> angular_velocity_x_buffer;
    std::vector<double> angular_velocity_y_buffer;
    std::vector<double> angular_velocity_z_buffer;
 
    // // PitchrateOffsetStopEstimator variables
    PitchrateOffsetStopEstimator pitchrate_offset_stop_estimator;
    double pitchrate_offset;
    Status pitchrate_offset_status;

    // PitchingEstimator variables
    PitchingEstimator pitching_estimator;
    double pitching;
    Status pitching_status;

    // AccelerationOffsetEstimator
    AccelerationOffsetEstimator acceleration_offset_estimator;    
    double filtered_acceleration;
    double acceleration_offset;
    Status acceleration_offset_status;

    //DopplerFusion
    double velocity;
    std::vector<double> time_buffer;
    std::vector<double> doppler_velocity_buffer;
    std::vector<double> corrected_acceleration_buffer;
    std::vector<double> rtklib_update_status_buffer;
    Status velocity_status;

    bool updateImu(const ImuState);
    bool updateGGA(const GNSSState);
    bool Stopjudgement(const ImuState);
    bool DopplerFusion();
};


#endif /*NAVIGATION_H */
