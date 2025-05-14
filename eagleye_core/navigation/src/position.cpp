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

/*
 * position.cpp
 * Author MapIV Sekino
 */

// This algorithm makes a naive estimate of the current position by projecting
// the input ENU velocity onto the current position estimate for the delta time between
// the last execution time and the ENU velocity timestamp

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

#ifndef DEBUG
#define DEBUG 0
#endif
#define debug_print(fmt, ...) do { if (DEBUG) { fprintf(stderr, fmt, __VA_ARGS__); }} while (0)

// The original implementation of the coordinate system transformation using ROS tf below.
// Replaced with Eigen to get rid of ROS dependencies
//    tf::Quaternion q;
//    q.setRPY(0, 0, kQuarterTurnRadians - heading_interpolate_3rd.heading_angle);
//
//    tf::Transform transform;
//    transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
//
//    transform.setRotation(q);
//
//    tf::Transform transform2;
//    tf::Quaternion q2(position_parameter.tf_gnss_rotation_x, position_parameter.tf_gnss_rotation_y, position_parameter.tf_gnss_rotation_z, position_parameter.tf_gnss_rotation_w);
//
//    transform2.setOrigin(transform*tf::Vector3(-position_parameter.tf_gnss_translation_x, -position_parameter.tf_gnss_translation_y, -position_parameter.tf_gnss_translation_z));
//    transform2.setRotation(transform*q2);
//
//    tf::Vector3 tmp_pos;
//    tmp_pos = transform2.getOrigin();*/

static inline Eigen::Isometry3d rotatePosition(double* enu_position, const double map_heading_rad) {

    // The code block below appears to relate to applying a correction to the GNSS easimate pose.
    // The source of this correction (aside from the Position algorithm settings) is unclear.
    // The eagleye_rt/src/position_node.cpp appears to get these from a tf2 transform broadcaster,
    // but it's unclear if one is ever used and what the purpose of this exercise is. Maybe for RTK use?
    // The implementation is also needlessly complex and causes a decent performance hit.
    constexpr double kQuarterTurnRadians{90.0 * M_PI / 180.0};
    constexpr double kHalfTurnRadians{2.0 * M_PI};

    // Unwind the heading angle (in radians) to 2Pi, equivalent to a range of 360 degrees
    const double unwound_mapheading_rad = fmod(map_heading_rad, kHalfTurnRadians);

    // Convert the True North map heading to an ENU heading. ENU headings wind in the opposite direction
    // and are offset by 90 degrees (pi/2 radians): zero is East rather than North.
    const double enu_heading{kQuarterTurnRadians - unwound_mapheading_rad};

    // Represent the ENU position as a vector
    PoseStamped pose;
    pose.pose.position.x = enu_position[0];
    pose.pose.position.y = enu_position[1];
    pose.pose.position.z = enu_position[2];
    Eigen::Vector3d base_position(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    // Create a quaternion from the yaw angle only
    Eigen::Quaterniond base_orientation_q(Eigen::AngleAxisd(enu_heading, Eigen::Vector3d::UnitZ()));
    // Rotate the ENU position vector using the quaternion
    Eigen::Isometry3d unadjusted_world_pose = Eigen::Translation3d(base_position) * base_orientation_q;
    return unadjusted_world_pose;
}

static inline Eigen::Vector3d applyAdjustmentTransformation(const Eigen::Isometry3d& unadjusted_world_position, const PositionParameter& config) {

    // The following appears to relate to a refinement of the estimated ENU position
    // Produce another Quaternion from a configured rotation (and normalize it
    Eigen::Quaterniond adjustment_rotation_relative(
        config.tf_gnss_rotation_w,
        config.tf_gnss_rotation_x,
        config.tf_gnss_rotation_y,
        config.tf_gnss_rotation_z
    );
    adjustment_rotation_relative.normalize();

    // Create a a translation from a configured position offset)
    Eigen::Vector3d adjustment_translation_relative_neg(
        -config.tf_gnss_translation_x,
        -config.tf_gnss_translation_y,
        -config.tf_gnss_translation_z
    );

    // Translate the rotated ENU position by the configured offsets from the configuration
    Eigen::Vector3d adjusted_position_world = unadjusted_world_position * adjustment_translation_relative_neg;
    // Rotate the heading quaternion with the configured rotation values to provduce a new rotation quaternion
    const Eigen::Quaterniond unadjusted_world_rotation(unadjusted_world_position.rotation());
    Eigen::Quaterniond adjusted_orientation_world = unadjusted_world_rotation * adjustment_rotation_relative;
    adjusted_orientation_world.normalize();

    // Rotate the translated ENU position with the new rotation quaternion that considers the configured rotation
    Eigen::Isometry3d adjusted_world_pose = Eigen::Translation3d(adjusted_position_world) * adjusted_orientation_world;

    // Get the new ENU position
    Eigen::Vector3d tmp_pos = adjusted_world_pose.translation();
    return tmp_pos;
}


void position_estimate_(TwistStamped velocity,StatusStamped velocity_status,Distance distance,
  Heading heading_interpolate_3rd,Vector3Stamped enu_vel,PositionParameter position_parameter,
  PositionStatus* position_status, Position* enu_absolute_pos)
{
  int i;
  int estimated_number_max = position_parameter.estimated_interval/position_parameter.update_distance;
  int max_x_index, max_y_index;
  double enu_pos[3];
  double avg_x, avg_y, avg_z;
  double tmp_enu_pos_x, tmp_enu_pos_y, tmp_enu_pos_z;
  bool data_status, gnss_status;
  std::size_t index_length;
  std::size_t velocity_index_length;
  std::vector<double> base_enu_pos_x_buffer, base_enu_pos_y_buffer, base_enu_pos_z_buffer;
  std::vector<double> diff_x_buffer2, diff_y_buffer2, diff_z_buffer2;
  std::vector<double> base_enu_pos_x_buffer2,  base_enu_pos_y_buffer2, base_enu_pos_z_buffer2;
  std::vector<double> diff_x_buffer, diff_y_buffer, diff_z_buffer;
  std::vector<double>::iterator max_x, max_y;

  double enabled_data_ratio = position_parameter.gnss_rate / position_parameter.imu_rate * position_parameter.gnss_receiving_threshold;
  double remain_data_ratio = enabled_data_ratio * position_parameter.outlier_ratio_threshold;

  // Take the ENU-converted GNSS measurement from the internal state
  enu_pos[0] = position_status->enu_pos[0];
  enu_pos[1] = position_status->enu_pos[1];
  enu_pos[2] = position_status->enu_pos[2];

  if (position_status->gnss_update_failure == false) {
    gnss_status = true;
    debug_print("GNSS status is set to %s\n", gnss_status ? "ON" : "OFF");
    // We rotate the ENU position for some reason, using the true north map heading estimate
    const Eigen::Isometry3d rotated_position = rotatePosition(&enu_pos[0], heading_interpolate_3rd.heading_angle);

    // If we have an adjustment rotation magnitude greater than zero, or an adjustment translation greater than zero, apply that adjustment
    const double config_translation_magnitude = std::fabs(position_parameter.tf_gnss_translation_x) + std::fabs(position_parameter.tf_gnss_translation_y) + std::fabs(position_parameter.tf_gnss_translation_z);
    if (std::fabs(position_parameter.tf_gnss_rotation_w) > std::numeric_limits<double>::epsilon() || config_translation_magnitude > std::numeric_limits<double>::epsilon()) {
        const Eigen::Vector3d adjusted_position = applyAdjustmentTransformation(rotated_position, position_parameter);
        enu_pos[0] = adjusted_position[0];
        enu_pos[1] = adjusted_position[1];
        enu_pos[2] = adjusted_position[2];
    } else { // Otherwise, just use the rotated position
        const Eigen::Vector3d unadjusted_position = rotated_position.translation();
        enu_pos[0] = unadjusted_position[0];
        enu_pos[1] = unadjusted_position[1];
        enu_pos[2] = unadjusted_position[2];
    }
  } else {
    debug_print("GNSS status is set to %s\n", position_status->gnss_update_failure ? "True" : "False");
    gnss_status = false;
  }

  // If we have seemingly valid input heading information and vehicle kinematics information, increment the input heading sample counter
  if (heading_interpolate_3rd.status.estimate_status == true && velocity_status.status.enabled_status == true) {
    heading_interpolate_3rd.status.estimate_status = false; //in order to prevent this data frame from being evaluated more than once
    ++position_status->heading_estimate_status_count;
    debug_print("interpolated heading status [ON], confidence level %d\n", position_status->heading_estimate_status_count);
  } else {
    debug_print("position_estimate_() Invalid data condition: heading interpolate [%s], velocity [%s]\n",
        heading_interpolate_3rd.status.estimate_status ? "ON" : "OFF",
        velocity_status.status.enabled_status ? "ON" : "OFF"
    );
  }

  const double enu_velocity_timestamp_seconds{enu_vel.header.stamp.tv_sec + enu_vel.header.stamp.tv_nsec * 1e-9};
  const double position_update_delta_t_seconds{enu_velocity_timestamp_seconds - position_status->time_last};
  if(position_status->time_last < std::numeric_limits<double>::epsilon()) { // If this is not the first execution cycle, i.e. we have not stored a last timestamp
    // Update the relative position estimate with the XYZ speeds multiplied by the delta time between the last update timestamp and the ENU velocity input timestamp
    // This requires that the unit for the ENU velocities is meters per second
    position_status->enu_relative_pos_x = position_status->enu_relative_pos_x + enu_vel.vector.x * position_update_delta_t_seconds;
    position_status->enu_relative_pos_y = position_status->enu_relative_pos_y + enu_vel.vector.y * position_update_delta_t_seconds;
    position_status->enu_relative_pos_z = position_status->enu_relative_pos_z + enu_vel.vector.z * position_update_delta_t_seconds;
  }

  // if the internal state heading sample count is greater than zero, the GNSS input seemed valid and the position update minimum distance condition is met..
  if (distance.distance - position_status->distance_last >= position_parameter.update_distance && gnss_status == true && position_status->heading_estimate_status_count > 0) {
    if (position_status->estimated_number < estimated_number_max) {
      ++position_status->estimated_number;
    } else {
      position_status->estimated_number = estimated_number_max;
    }

    // Push the ENU position estimate (that seems to be just the GNSS measurement really) into an internal state buffer
    position_status->enu_pos_x_buffer.push_back(enu_pos[0]);
    position_status->enu_pos_y_buffer.push_back(enu_pos[1]);
    //enu_pos_z_buffer.push_back(enu_pos[2]);
    position_status->enu_pos_z_buffer.push_back(0);  // Someone's hack; disabled Z axis

    position_status->correction_velocity_buffer.push_back(velocity.twist.linear.x);

    position_status->enu_relative_pos_x_buffer.push_back(position_status->enu_relative_pos_x);
    position_status->enu_relative_pos_y_buffer.push_back(position_status->enu_relative_pos_y);
    //position_status->enu_relative_pos_z_buffer.push_back(position_status->enu_relative_pos_z);
    position_status->enu_relative_pos_z_buffer.push_back(0);  // Someone's hack; disabled Z axis

    position_status->distance_buffer.push_back(distance.distance);

    data_status = true; // set data status to true, as we have refreshed the internal state data inputs

    // If the internal state distance measurement buffer is overflowing, pop one entry from all buffers
    if (position_status->distance_buffer.end() - position_status->distance_buffer.begin() > estimated_number_max) {
      position_status->enu_pos_x_buffer.erase(position_status->enu_pos_x_buffer.begin());
      position_status->enu_pos_y_buffer.erase(position_status->enu_pos_y_buffer.begin());
      position_status->enu_pos_z_buffer.erase(position_status->enu_pos_z_buffer.begin());
      position_status->correction_velocity_buffer.erase(position_status->correction_velocity_buffer.begin());
      position_status->enu_relative_pos_x_buffer.erase(position_status->enu_relative_pos_x_buffer.begin());
      position_status->enu_relative_pos_y_buffer.erase(position_status->enu_relative_pos_y_buffer.begin());
      position_status->enu_relative_pos_z_buffer.erase(position_status->enu_relative_pos_z_buffer.begin());
      position_status->distance_buffer.erase(position_status->distance_buffer.begin());
    }
    position_status->distance_last = distance.distance;
  }

  // If we flagged that we have new (valid) data
  if (data_status == true) {
    debug_print("GNSS Data conditions met, estimating position...%s\n", "yay");
    // If we have moved a configured minimum interval, the GNSS measurement appears valid, the vehicle appears to be moving and we have
    // at least one input heading sample in the internal state, ...
    if (distance.distance > position_parameter.estimated_interval && gnss_status == true &&
        velocity.twist.linear.x > position_parameter.moving_judgement_threshold && position_status->heading_estimate_status_count > 0)
    {
      std::vector<int> distance_index;
      std::vector<int> velocity_index;
      std::vector<int> index;

      for (i = 0; i < position_status->estimated_number; i++) {
        if (position_status->distance_buffer[position_status->estimated_number-1] - position_status->distance_buffer[i]  <= position_parameter.estimated_interval) {
          distance_index.push_back(i);
          if (position_status->correction_velocity_buffer[i] > position_parameter.moving_judgement_threshold) {
            velocity_index.push_back(i);
          }
        }
      }

      set_intersection(velocity_index.begin(), velocity_index.end(), distance_index.begin(), distance_index.end(), inserter(index, index.end()));

      index_length = std::distance(index.begin(), index.end());
      velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

      if (index_length > velocity_index_length * enabled_data_ratio) {
        while (true) {
          index_length = std::distance(index.begin(), index.end());

          base_enu_pos_x_buffer.clear();
          base_enu_pos_y_buffer.clear();
          base_enu_pos_z_buffer.clear();

          for (i = 0; i < position_status->estimated_number; i++) {
            base_enu_pos_x_buffer.push_back(
                position_status->enu_pos_x_buffer[index[index_length-1]] -
                position_status->enu_relative_pos_x_buffer[index[index_length-1]] + position_status->enu_relative_pos_x_buffer[i]
            );
            base_enu_pos_y_buffer.push_back(
                position_status->enu_pos_y_buffer[index[index_length-1]] -
                position_status->enu_relative_pos_y_buffer[index[index_length-1]] + position_status->enu_relative_pos_y_buffer[i]
            );
            base_enu_pos_z_buffer.push_back(
                position_status->enu_pos_z_buffer[index[index_length-1]] - 
                position_status->enu_relative_pos_z_buffer[index[index_length-1]] + position_status->enu_relative_pos_z_buffer[i]
            );
          }

          diff_x_buffer2.clear();
          diff_y_buffer2.clear();
          diff_z_buffer2.clear();

          for (i = 0; i < static_cast<int>(index_length); i++) {
            diff_x_buffer2.push_back(base_enu_pos_x_buffer[index[i]] - position_status->enu_pos_x_buffer[index[i]]);
            diff_y_buffer2.push_back(base_enu_pos_y_buffer[index[i]] - position_status->enu_pos_y_buffer[index[i]]);
            diff_z_buffer2.push_back(base_enu_pos_z_buffer[index[i]] - position_status->enu_pos_z_buffer[index[i]]);
          }

          avg_x = std::accumulate(diff_x_buffer2.begin(), diff_x_buffer2.end(), 0.0) / index_length;
          avg_y = std::accumulate(diff_y_buffer2.begin(), diff_y_buffer2.end(), 0.0) / index_length;
          avg_z = std::accumulate(diff_z_buffer2.begin(), diff_z_buffer2.end(), 0.0) / index_length;

          tmp_enu_pos_x = position_status->enu_pos_x_buffer[index[index_length - 1]] - avg_x;
          tmp_enu_pos_y = position_status->enu_pos_y_buffer[index[index_length - 1]] - avg_y;
          tmp_enu_pos_z = position_status->enu_pos_z_buffer[index[index_length - 1]] - avg_z;

          base_enu_pos_x_buffer2.clear();
          base_enu_pos_y_buffer2.clear();
          base_enu_pos_z_buffer2.clear();

          for (i = 0; i < position_status->estimated_number; i++) {
            base_enu_pos_x_buffer2.push_back(
                tmp_enu_pos_x - position_status->enu_relative_pos_x_buffer[index[index_length - 1]] +
                position_status->enu_relative_pos_x_buffer[i]
            );
            base_enu_pos_y_buffer2.push_back(
                tmp_enu_pos_y - position_status->enu_relative_pos_y_buffer[index[index_length - 1]] +
                position_status->enu_relative_pos_y_buffer[i]
            );
            base_enu_pos_z_buffer2.push_back(
                tmp_enu_pos_z - position_status->enu_relative_pos_z_buffer[index[index_length - 1]] +
                position_status->enu_relative_pos_z_buffer[i]
            );
          }

          diff_x_buffer.clear();
          diff_y_buffer.clear();
          diff_z_buffer.clear();

          for (i = 0; i < static_cast<int>(index_length); i++) {
            diff_x_buffer.push_back(fabsf(base_enu_pos_x_buffer2[index[i]] - position_status->enu_pos_x_buffer[index[i]]));
            diff_y_buffer.push_back(fabsf(base_enu_pos_y_buffer2[index[i]] - position_status->enu_pos_y_buffer[index[i]]));
            diff_z_buffer.push_back(fabsf(base_enu_pos_z_buffer2[index[i]] - position_status->enu_pos_z_buffer[index[i]]));
          }

          max_x = std::max_element(diff_x_buffer.begin(), diff_x_buffer.end());
          max_y = std::max_element(diff_y_buffer.begin(), diff_y_buffer.end());

          max_x_index = std::distance(diff_x_buffer.begin(), max_x);
          max_y_index = std::distance(diff_y_buffer.begin(), max_y);

          if(diff_x_buffer[max_x_index] < diff_y_buffer[max_y_index]) {
            if (diff_x_buffer[max_x_index] > position_parameter.outlier_threshold) {
              index.erase(index.begin() + max_x_index);
            } else {
              break;
            }
          } else {
            if (diff_y_buffer[max_y_index] > position_parameter.outlier_threshold) {
              index.erase(index.begin() + max_y_index);
            } else {
              break;
            }
          }

          index_length = std::distance(index.begin(), index.end());
          velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

          if (index_length < velocity_index_length * remain_data_ratio) {
            break;
          }
        }

        index_length = std::distance(index.begin(), index.end());
        velocity_index_length = std::distance(velocity_index.begin(), velocity_index.end());

        if (index_length >= velocity_index_length * remain_data_ratio) {
          std::vector<double> diff_x_buffer_for_covariance, diff_y_buffer_for_covariance, diff_z_buffer_for_covariance;
          for (i = 0; i < static_cast<int>(index_length); i++) {
            diff_x_buffer_for_covariance.push_back(base_enu_pos_x_buffer2[index[i]] - position_status->enu_pos_x_buffer[index[i]]);
            diff_y_buffer_for_covariance.push_back(base_enu_pos_y_buffer2[index[i]] - position_status->enu_pos_y_buffer[index[i]]);
            diff_z_buffer_for_covariance.push_back(base_enu_pos_z_buffer2[index[i]] - position_status->enu_pos_z_buffer[index[i]]);
          }

          avg_x = std::accumulate(diff_x_buffer_for_covariance.begin(), diff_x_buffer_for_covariance.end(), 0.0) / index_length;
          avg_y = std::accumulate(diff_y_buffer_for_covariance.begin(), diff_y_buffer_for_covariance.end(), 0.0) / index_length;
          avg_z = std::accumulate(diff_z_buffer_for_covariance.begin(), diff_z_buffer_for_covariance.end(), 0.0) / index_length;

          double cov_x, cov_y, cov_z;
          double square_sum_x = 0, square_sum_y = 0, square_sum_z = 0;
          for (i = 0; i < static_cast<int>(index_length); i++) {
            square_sum_x += (diff_x_buffer_for_covariance[i] - avg_x) * (diff_x_buffer_for_covariance[i] - avg_x);
            square_sum_y += (diff_y_buffer_for_covariance[i] - avg_y) * (diff_y_buffer_for_covariance[i] - avg_y);
            square_sum_z += (diff_z_buffer_for_covariance[i] - avg_z) * (diff_z_buffer_for_covariance[i] - avg_z);
          }
          cov_x = square_sum_x/index_length;
          cov_y = square_sum_y/index_length;
          cov_z = square_sum_z/index_length;

          if (index[index_length - 1] == position_status->estimated_number-1) {
            enu_absolute_pos->enu_pos.x = tmp_enu_pos_x;
            enu_absolute_pos->enu_pos.y = tmp_enu_pos_y;
            enu_absolute_pos->enu_pos.z = tmp_enu_pos_z;
            enu_absolute_pos->covariance[0] = cov_x + position_parameter.gnss_error_covariance; // [m^2]
            enu_absolute_pos->covariance[4] = cov_y + position_parameter.gnss_error_covariance; // [m^2]
            enu_absolute_pos->covariance[8] = cov_z + position_parameter.gnss_error_covariance; // [m^2]
            enu_absolute_pos->status.enabled_status = true;
            enu_absolute_pos->status.estimate_status = true;
          }
        }
      }
    }
  } else {
    debug_print("GNSS Data conditions failed (%s), no position estimate.\n", "err");
  }
  // Set the internal state last update time to the input ENU velocity timestamp
  position_status->time_last = enu_velocity_timestamp_seconds;
  data_status = false;
}

void position_estimate(GNSSState gga,TwistStamped velocity,StatusStamped velocity_status,
  Distance distance,Heading heading_interpolate_3rd,Vector3Stamped enu_vel,
  PositionParameter position_parameter,PositionStatus* position_status,Position* enu_absolute_pos)
{
  double llh_pos[3];
  double enu_pos[3];
  double ecef_pos[3];
  double ecef_base_pos[3];
  bool gnss_update_failure;

  constexpr double deg2rad{M_PI/180.0};

  // Convert the GNSS measurement geodetic coordinates to ECEF
  llh_pos[0] = gga.position.lat * deg2rad;
  llh_pos[1] = gga.position.lon * deg2rad;
  llh_pos[2] = gga.position.alt_msl; //  + gga.undulation;
  llh2xyz(llh_pos,ecef_pos);

  // If no ECEF base position is set yet...
  if(enu_absolute_pos->ecef_base_pos.x == 0 && enu_absolute_pos->ecef_base_pos.y == 0 && enu_absolute_pos->ecef_base_pos.z == 0) {
    // If we have a GNSS measurement, set the base position to the ECEF version of the GNSS position
    if (gga.timestamp_ns != 0) {
      enu_absolute_pos->ecef_base_pos.x = ecef_pos[0];
      enu_absolute_pos->ecef_base_pos.y = ecef_pos[1];
      enu_absolute_pos->ecef_base_pos.z = ecef_pos[2];
      // If the user has configured an ECEF base position override, ignore the above and use the config setting instead
      if(position_parameter.ecef_base_pos_x != 0 && position_parameter.ecef_base_pos_y != 0 && position_parameter.ecef_base_pos_z != 0) {
        enu_absolute_pos->ecef_base_pos.x = position_parameter.ecef_base_pos_x;
        enu_absolute_pos->ecef_base_pos.y = position_parameter.ecef_base_pos_y;
        enu_absolute_pos->ecef_base_pos.z = position_parameter.ecef_base_pos_z;
      }
    }
    debug_print("Set ECEF base position to x=%3.9f, y=%3.9f, z=%3.9f\n",
        enu_absolute_pos->ecef_base_pos.x,
        enu_absolute_pos->ecef_base_pos.y,
        enu_absolute_pos->ecef_base_pos.z
    );
  }

  // Convert the GNSS measurement ECEF to ENU, using whatever current ECEF origin values
  ecef_base_pos[0] = enu_absolute_pos->ecef_base_pos.x;
  ecef_base_pos[1] = enu_absolute_pos->ecef_base_pos.y;
  ecef_base_pos[2] = enu_absolute_pos->ecef_base_pos.z;
  xyz2enu(ecef_pos, ecef_base_pos, enu_pos);

  // If the conversion result is numerically invalid, set the ENU position to zero
  if (!std::isfinite(enu_pos[0])||!std::isfinite(enu_pos[1])||!std::isfinite(enu_pos[2])) {
    debug_print("ECEF to ENU conversion result failure: lat=%3.9f, lon=%3.9f\n",
        enu_pos[0],
        enu_pos[1]
    );
    enu_pos[0] = 0.0;
    enu_pos[1] = 0.0;
    enu_pos[2] = 0.0;
    gnss_update_failure = true;
  } else {
    gnss_update_failure = false;
  }

  // If we have already processed the input GNSS measurement (identified by timestamp) or the ENU velocity header has an invalid timestamp,
  // invalidate the ENU position and flag a GNSS update failure.
  const double gnss_update_timestamp_seconds{gga.timestamp_ns * 1e-9};
  if (position_status->nmea_time_last == gnss_update_timestamp_seconds || enu_vel.header.stamp.tv_sec == 0) {
    debug_print(" GNSS timestamp is seen before [%d] or ENU velocity stamp is zero [%d]. Flagging input data as invalid.\n",
        (position_status->nmea_time_last == gnss_update_timestamp_seconds),
        enu_vel.header.stamp.tv_sec == 0);
    enu_pos[0] = 0.0;
    enu_pos[1] = 0.0;
    enu_pos[2] = 0.0;
    gnss_update_failure = true;
  }

  // Update the internal state with the new ENU position estimate (no matter if valid or not)
  position_status->enu_pos[0] = enu_pos[0];
  position_status->enu_pos[1] = enu_pos[1];
  position_status->enu_pos[2] = enu_pos[2];
  position_status->gnss_update_failure = gnss_update_failure;  // Register the GNSS update failure flag in the internal state
  position_status->nmea_time_last = gnss_update_timestamp_seconds;  // Register the GNSS update timestamp in the internal state
  // Run the private estimator algorithm
  position_estimate_(velocity, velocity_status, distance, heading_interpolate_3rd, enu_vel, position_parameter, position_status, enu_absolute_pos);
}
