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
 * position_interpolate.cpp
 * Author MapIV Sekino
 */

// This position estimator algorithm uses the ENU velocity estimate's timestamp
// to determine the time point to which to estimate. Therefore, to predict forward,
// an ENU velocity estimate with a timestmap in the future needs to be provided as input.

#include "coordinate/coordinate.hpp"
#include "navigation/navigation.hpp"

// 3-dimensional pythagorean theorem to calculate 3-dimensional speed from the XYZ velocity vector
static inline double hypotenuse_pyth3d(double leg_x, double leg_y, double leg_z) {
  return std::sqrt(
    (leg_x * leg_x) + (leg_y * leg_y) + (leg_z * leg_z)
  );
}


void position_interpolate_estimate(Position enu_absolute_pos, Vector3Stamped enu_vel, Position gnss_smooth_pos,
  Height height, Heading heading,PositionInterpolateParameter position_interpolate_parameter, PositionInterpolateStatus* position_interpolate_status,
  Position* enu_absolute_pos_interpolate, GNSSPosition* eagleye_fix)
{

  int i;
  int estimate_index = 0;
  double enu_pos[3],tmp_enu[3];
  double ecef_base_pos[3];
  double ecef_pos[3];
  double llh_pos[3];
  double diff_estimate_enu_pos_x = 0.0;
  double diff_estimate_enu_pos_y = 0.0;
  double diff_estimate_enu_pos_z = 0.0;
  bool position_estimate_status;

  const double search_buffer_number = position_interpolate_parameter.sync_search_period * position_interpolate_parameter.imu_rate;

  enu_absolute_pos_interpolate->ecef_base_pos = enu_absolute_pos.ecef_base_pos;

  if (position_interpolate_status->number_buffer < search_buffer_number) {
    ++position_interpolate_status->number_buffer;
  } else {
    position_interpolate_status->number_buffer = search_buffer_number;
  }

  const double enu_absolute_time = enu_absolute_pos.header.stamp.tv_sec + enu_absolute_pos.header.stamp.tv_nsec * 1e-9;
  // If the internal state last timestamp is not equal to the input Position measurement and the Position measurement is marked as valid
  if (position_interpolate_status->position_stamp_last != enu_absolute_time && enu_absolute_pos.status.estimate_status == true) {
    // Increase the confidence in the internal state and mark the position interpolation status to started
    position_estimate_status = true;
    position_interpolate_status->position_estimate_start_status = true;
    ++position_interpolate_status->position_estimate_status_count;
  } else {
    position_estimate_status = false;
  }

  // Use the last ENU velocity's timestamp as the timestamp to estimate towards.
  // To predict forward, the ENU velocity estimate needs to be timestamped in in the future
  const double enu_vel_time = enu_vel.header.stamp.tv_sec + enu_vel.header.stamp.tv_nsec / 1e9;
  const double enu_3d_speed = hypotenuse_pyth3d(enu_vel.vector.x, enu_vel.vector.y, enu_vel.vector.z);
  bool has_new_naive_estimate{false};
  // If this is not the first execution cycle and if our current ENU velocity is high enough to look like vehicle movement...
  if(position_interpolate_status->time_last > std::numeric_limits<double>::epsilon() && enu_3d_speed > position_interpolate_parameter.stop_judgement_threshold) {
    const double delta_t{enu_vel_time - position_interpolate_status->time_last};
    // Set the provisional position to be the last execution cycle projected position adjusted by our current speed and delta t
    debug_print("    Using previous position estimate [%4.9f][%4.9f][%4.9f] from %f seconds ago (enu ts %f - last %f\n",
        position_interpolate_status->provisional_enu_pos_x,
        position_interpolate_status->provisional_enu_pos_y,
        position_interpolate_status->provisional_enu_pos_z,
        delta_t,
        enu_vel_time,
        position_interpolate_status->time_last
    );
    position_interpolate_status->provisional_enu_pos_x = enu_absolute_pos_interpolate->enu_pos.x + enu_vel.vector.x * delta_t;
    position_interpolate_status->provisional_enu_pos_y = enu_absolute_pos_interpolate->enu_pos.y + enu_vel.vector.y * delta_t;
    position_interpolate_status->provisional_enu_pos_z = enu_absolute_pos_interpolate->enu_pos.z + enu_vel.vector.z * delta_t;
    debug_print("    Naive new position estimate is [%4.9f][%4.9f][%4.9f] based on ENU velocity [%3.1f][%3.1f][%3.1f]\n",
        position_interpolate_status->provisional_enu_pos_x,
        position_interpolate_status->provisional_enu_pos_y,
        position_interpolate_status->provisional_enu_pos_z,
        enu_vel.vector.x,
        enu_vel.vector.y,
        enu_vel.vector.z
    );
    has_new_naive_estimate = true;
  }

  // Push the provisional ENU position into the provisional position buffer
  position_interpolate_status->provisional_enu_pos_x_buffer.push_back(position_interpolate_status->provisional_enu_pos_x);
  position_interpolate_status->provisional_enu_pos_y_buffer.push_back(position_interpolate_status->provisional_enu_pos_y);
  position_interpolate_status->provisional_enu_pos_z_buffer.push_back(position_interpolate_status->provisional_enu_pos_z);
  position_interpolate_status->imu_stamp_buffer.push_back(enu_vel_time);
  if (has_new_naive_estimate) {
      debug_print("    Pushed naive position estimate to history buffer with ENU velocity timestamp as key%s", "\n");
      has_new_naive_estimate = false;
  } else {
      debug_print("    Pushed old position estimate to history buffer with ENU velocity timestamp as key%s", "\n");
  }
  const size_t imu_stamp_buffer_length = std::distance(position_interpolate_status->imu_stamp_buffer.begin(), position_interpolate_status->imu_stamp_buffer.end());
  // If the provisional position buffer has too much data, clean out old data
  if (imu_stamp_buffer_length > search_buffer_number) {
    position_interpolate_status->provisional_enu_pos_x_buffer.erase(position_interpolate_status->provisional_enu_pos_x_buffer.begin());
    position_interpolate_status->provisional_enu_pos_y_buffer.erase(position_interpolate_status->provisional_enu_pos_y_buffer.begin());
    position_interpolate_status->provisional_enu_pos_z_buffer.erase(position_interpolate_status->provisional_enu_pos_z_buffer.begin());
    position_interpolate_status->imu_stamp_buffer.erase(position_interpolate_status->imu_stamp_buffer.begin());
  }

  // If we have received a valid input Position measurement at least once, maybe even during this cycle...
  if (position_interpolate_status->position_estimate_start_status == true) {
    // If this position interpolation estimate should be performed
    if (position_estimate_status == true) {
      // Iterate over the IMU timestamp buffer until we find a timestamp that matches the input Position measurement timestamp
      for (estimate_index = position_interpolate_status->number_buffer; estimate_index > 0; estimate_index--) {
        if (position_interpolate_status->imu_stamp_buffer[estimate_index-1] == enu_absolute_time) {
          debug_print("    Found ENU Velocity timestamp in history that matches Position measurement at index %d\n", estimate_index-1);
          break;
        }
      }
    }

    // If we found a match between the IMU timestamp buffer entry and the input Position measurement
    // and if we have enough data in the history buffer and we have interpolated before...
    if (position_estimate_status == true && estimate_index > 0 && position_interpolate_status->number_buffer >= estimate_index &&
      position_interpolate_status->position_estimate_status_count > 1)
    {
      // Calculate the ENU position delta between the provisional ENU position in the history buffer and the input Position measurement
      diff_estimate_enu_pos_x = (position_interpolate_status->provisional_enu_pos_x_buffer[estimate_index-1] - enu_absolute_pos.enu_pos.x);
      diff_estimate_enu_pos_y = (position_interpolate_status->provisional_enu_pos_y_buffer[estimate_index-1] - enu_absolute_pos.enu_pos.y);
      diff_estimate_enu_pos_z = (position_interpolate_status->provisional_enu_pos_z_buffer[estimate_index-1] - enu_absolute_pos.enu_pos.z);
      debug_print("    Position interpolation estimate history delta from input Position: [%4.9f][%4.9f][%4.9f]\n",
        diff_estimate_enu_pos_x,
        diff_estimate_enu_pos_y,
        diff_estimate_enu_pos_z
      );
      // Adjust the provisional ENU position history with the offset between the estimate and the Position measurement
      for (i = estimate_index; i <= position_interpolate_status->number_buffer; i++) {
        position_interpolate_status->provisional_enu_pos_x_buffer[i-1] -= diff_estimate_enu_pos_x;
        position_interpolate_status->provisional_enu_pos_y_buffer[i-1] -= diff_estimate_enu_pos_y;
        position_interpolate_status->provisional_enu_pos_z_buffer[i-1] -= diff_estimate_enu_pos_z;
      }
      // Set the provisional ENU position to be the last entry of the position buffer
      position_interpolate_status->provisional_enu_pos_x = position_interpolate_status->provisional_enu_pos_x_buffer[position_interpolate_status->number_buffer-1];
      position_interpolate_status->provisional_enu_pos_y = position_interpolate_status->provisional_enu_pos_y_buffer[position_interpolate_status->number_buffer-1];
      position_interpolate_status->provisional_enu_pos_z = position_interpolate_status->provisional_enu_pos_z_buffer[position_interpolate_status->number_buffer-1];
      debug_print("    New ENU position estimate [%4.9f][%4.9f][%4.9f]\n",
            position_interpolate_status->provisional_enu_pos_x,
            position_interpolate_status->provisional_enu_pos_y,
            position_interpolate_status->provisional_enu_pos_z
      );
      position_interpolate_status->is_estimate_start = true;
      enu_absolute_pos_interpolate->status.enabled_status = true;
      enu_absolute_pos_interpolate->status.estimate_status = true;
    }
    else if (position_interpolate_status->position_estimate_status_count == 1) {
      // If we are only getting started, set the provisional ENU position to just be equal to the input Position measurement
      position_interpolate_status->provisional_enu_pos_x = enu_absolute_pos.enu_pos.x;
      position_interpolate_status->provisional_enu_pos_y = enu_absolute_pos.enu_pos.y;
      position_interpolate_status->provisional_enu_pos_z = enu_absolute_pos.enu_pos.z;
      enu_absolute_pos_interpolate->status.enabled_status = true;
      enu_absolute_pos_interpolate->status.estimate_status = false;
    } else {
      // If something went wrong earlier, just set the interpolated position estimate status to false
      enu_absolute_pos_interpolate->status.estimate_status = false;
    }
  }

  if (position_interpolate_status->position_estimate_start_status && position_interpolate_status->is_estimate_start) {

    // Convert the interpolated ENU provisional position to Lat-Lon-Alt
    enu_pos[0] = position_interpolate_status->provisional_enu_pos_x;
    enu_pos[1] = position_interpolate_status->provisional_enu_pos_y;
    //enu_pos[2] = position_interpolate_status->provisional_enu_pos_z;
    enu_pos[2] = gnss_smooth_pos.enu_pos.z;
    ecef_base_pos[0] = enu_absolute_pos.ecef_base_pos.x;
    ecef_base_pos[1] = enu_absolute_pos.ecef_base_pos.y;
    ecef_base_pos[2] = enu_absolute_pos.ecef_base_pos.z;
    enu2llh(enu_pos, ecef_base_pos, llh_pos);

    // Calculate the covariances for the position
    Eigen::MatrixXd init_covariance;
    init_covariance = Eigen::MatrixXd::Zero(6, 6);
    init_covariance(0,0) = enu_absolute_pos.covariance[0];
    init_covariance(1,1) = enu_absolute_pos.covariance[4];
    init_covariance(2,2) = enu_absolute_pos.covariance[8];
    init_covariance(5,5) = heading.variance;
    double proc_noise = position_interpolate_parameter.proc_noise;
    Eigen::MatrixXd proc_covariance;
    proc_covariance = Eigen::MatrixXd::Zero(6, 6);
    proc_covariance(0,0) = proc_noise * proc_noise;
    proc_covariance(1,1) = proc_noise * proc_noise;
    proc_covariance(2,2) = proc_noise * proc_noise;
    Eigen::MatrixXd position_covariance;
    position_covariance = Eigen::MatrixXd::Zero(6, 6);
    double speed = std::sqrt(enu_vel.vector.x*enu_vel.vector.x + enu_vel.vector.y*enu_vel.vector.y + enu_vel.vector.z*enu_vel.vector.z);
    if(enu_absolute_pos_interpolate->status.estimate_status) {
      position_covariance = init_covariance;
      position_interpolate_status->position_covariance_last = position_covariance;
    } else if (speed > position_interpolate_parameter.stop_judgement_threshold) {
      const double delta_t{enu_vel_time - position_interpolate_status->time_last};
      Eigen::MatrixXd jacobian;
      jacobian = Eigen::MatrixXd::Zero(6, 6);
      jacobian(0,0) = 1;
      jacobian(1,1) = 1;
      jacobian(2,2) = 1;
      jacobian(3,3) = 1;
      jacobian(4,4) = 1;
      jacobian(5,5) = 1;
      jacobian(0,5) = enu_vel.vector.y * delta_t;
      jacobian(1,5) = -enu_vel.vector.x * delta_t;

      // MEMO: Jacobean not included
      // position_covariance = position_interpolate_status->position_covariance_last + proc_covariance;

      // MEMO: Jacobean not included
      position_covariance = jacobian * position_interpolate_status->position_covariance_last * (jacobian.transpose())   + proc_covariance;

      position_interpolate_status->position_covariance_last = position_covariance;
    } else {
      position_covariance = position_interpolate_status->position_covariance_last;
    }

    // Convert the Lat-Lon values from radians to degrees
    eagleye_fix->lon = llh_pos[1] * 180/M_PI;
    eagleye_fix->lat = llh_pos[0] * 180/M_PI;

    // If we have an altitude estimate, use that for the altitude
    if(height.status.enabled_status == true){
      llh_pos[2] = height.height;

      llh2xyz(llh_pos, ecef_pos);
      xyz2enu(ecef_pos, ecef_base_pos, tmp_enu);

      enu_pos[2] =  tmp_enu[2];
    }

    // Set the output ENU position and covariances
    enu_absolute_pos_interpolate->enu_pos.x = enu_pos[0];
    enu_absolute_pos_interpolate->enu_pos.y = enu_pos[1];
    enu_absolute_pos_interpolate->enu_pos.z = enu_pos[2];
    enu_absolute_pos_interpolate->covariance[0] = position_covariance(0,0); // [m^2]
    enu_absolute_pos_interpolate->covariance[4] = position_covariance(1,1); // [m^2]
    enu_absolute_pos_interpolate->covariance[8] = position_covariance(2,2); // [m^2]

    eagleye_fix->alt_msl = llh_pos[2];
    eagleye_fix->surface_stddev_m = position_covariance(0,0); // [m^2]
    eagleye_fix->alt_stddev_m = position_covariance(2,2); // [m^2]
  } else {
    enu_absolute_pos_interpolate->enu_pos.x = 0;
    enu_absolute_pos_interpolate->enu_pos.y = 0;
    enu_absolute_pos_interpolate->enu_pos.z = 0;
    eagleye_fix->lon = 0;
    eagleye_fix->lat = 0;
    eagleye_fix->alt_msl = 0;
    eagleye_fix->surface_stddev_m = 100;
    eagleye_fix->alt_stddev_m = 100;
  }

  position_interpolate_status->time_last = enu_vel_time;
  position_interpolate_status->position_stamp_last = enu_absolute_time;
}
