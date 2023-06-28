/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/common/trajectory_stitcher.h"

#include <algorithm>

#include "absl/strings/str_cat.h"

#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_model/vehicle_model.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleModel;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;

TrajectoryPoint TrajectoryStitcher::ComputeTrajectoryPointFromVehicleState(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  TrajectoryPoint point;
  point.mutable_path_point()->set_s(0.0);
  point.mutable_path_point()->set_x(vehicle_state.x());
  point.mutable_path_point()->set_y(vehicle_state.y());
  point.mutable_path_point()->set_z(vehicle_state.z());
  point.mutable_path_point()->set_theta(vehicle_state.heading());
  point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  point.set_v(vehicle_state.linear_velocity());
  point.set_a(vehicle_state.linear_acceleration());
  point.set_relative_time(planning_cycle_time);
  return point;
}

std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double planning_cycle_time, const VehicleState& vehicle_state) {
  TrajectoryPoint reinit_point;
  static constexpr double kEpsilon_v = 0.1;
  static constexpr double kEpsilon_a = 0.4;
  // TODO(Jinyun/Yu): adjust kEpsilon if corrected IMU acceleration provided
  if (std::abs(vehicle_state.linear_velocity()) < kEpsilon_v &&
      std::abs(vehicle_state.linear_acceleration()) < kEpsilon_a) {
    reinit_point = ComputeTrajectoryPointFromVehicleState(planning_cycle_time,
                                                          vehicle_state);
  } else {
    VehicleState predicted_vehicle_state;
    predicted_vehicle_state =
        VehicleModel::Predict(planning_cycle_time, vehicle_state);
    reinit_point = ComputeTrajectoryPointFromVehicleState(
        planning_cycle_time, predicted_vehicle_state);
  }

  return std::vector<TrajectoryPoint>(1, reinit_point);
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    PublishableTrajectory* prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(prev_trajectory->begin(), prev_trajectory->end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](common::TrajectoryPoint& p) {
                  auto x = p.path_point().x();
                  auto y = p.path_point().y();
                  auto theta = p.path_point().theta();

                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new =
                      common::math::NormalizeAngle(theta - theta_diff);

                  p.mutable_path_point()->set_x(x_new);
                  p.mutable_path_point()->set_y(y_new);
                  p.mutable_path_point()->set_theta(theta_new);
                });
}

/* Planning from current vehicle state if:
   1. the auto-driving mode is off
   (or) 2. we don't have the trajectory from last planning cycle
   (or) 3. the position deviation from actual and target is too high
*/
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time, const size_t preserved_points_num,       //preserved_points_num：默认20个点
    const bool replan_by_offset, const PublishableTrajectory* prev_trajectory,  //replan_by_offset：默认为TRUE
    std::string* replan_reason) {
  if (!FLAGS_enable_trajectory_stitcher) {        //FLAGS_enable_trajectory_stitcher:默认为TRUE，如果为false就会重规划
    *replan_reason = "stitch is disabled by gflag.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);   //如果重新规划，拼接的轨迹当然只有一个点作为规划的起点
  }
  if (!prev_trajectory) {     //缝合的前提是得有上一帧轨迹，如果没有就会重规划
    *replan_reason = "replan for no previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {     //COMPLETE_AUTO_DRIVE：完全自动驾驶默认，使能横向纵向
    *replan_reason = "replan for manual mode.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);   //这个返回的数据为重规划的起点，为什么？难道后边还有处理
  }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {   //之前有轨迹，但是轨迹点为0的话，仍然需要重新规划
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    *replan_reason = "replan for empty previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  const double veh_rel_time =         //计算当前时间和上一帧轨迹时间戳的差
      current_timestamp - prev_trajectory->header_time();

  size_t time_matched_index =            //从上一帧的轨迹中查找当前时间应该对应的index
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  if (time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {  //如果匹配的时间的点为0且相差的时间比第一个点的当前时间还要小，其实就是veh_rel_time小于0
    AWARN << "current time smaller than the previous trajectory's first time";
    *replan_reason =
        "replan for current time smaller than the previous trajectory's first "
        "time.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  if (time_matched_index + 1 >= prev_trajectory_size) {   //当前时间大于上一帧的最后一个点的时间，这个时候只能重规划了
    AWARN << "current time beyond the previous trajectory's last time";
    *replan_reason =
        "replan for current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  auto time_matched_point = prev_trajectory->TrajectoryPointAt(  //根据之前算出来的time_matched_index把这个匹配点找出来
      static_cast<uint32_t>(time_matched_index));

  if (!time_matched_point.has_path_point()) {  //检查这个匹配点是否有path_point字段，没有的话只能重规划了，这个怎么可能没有呢
    *replan_reason = "replan for previous trajectory missed path point";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(  //按照位置去查找，buffer的目的是为了避免相等的情况
      {vehicle_state.x(), vehicle_state.y()}, 1.0e-6);

  auto frenet_sd = ComputePositionProjection(  //计算自车位置在之前轨迹上的投影点的sd，注意这个地方是需要位置投影点，其实是中间量
      vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index)));

  if (replan_by_offset) {  //调用传入的为TRUE
    auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;   //匹配点和位置投影点纵向的误差：时间匹配点；注意frenet_sd.first是指车自身位置投影点的s值
    auto lat_diff = frenet_sd.second;  //横向误差

    ADEBUG << "Control lateral diff: " << lat_diff
           << ", longitudinal diff: " << lon_diff;

    if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lat_diff = ",
          lat_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,   //横向超出0.5，触发重规划
                                              vehicle_state);
    }

    if (std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {   //FLAGS_replan_longitudinal_distance_threshold：2.5
      const std::string msg = absl::StrCat(
          "the distance between matched point and actual position is too "
          "large. Replan is triggered. lon_diff = ",
          lon_diff);
      AERROR << msg;
      *replan_reason = msg;
      return ComputeReinitStitchingTrajectory(planning_cycle_time,   //纵向误差大于2.5触发重规划
                                              vehicle_state);
    }
  } else {
    ADEBUG << "replan according to certain amount of lat and lon offset is "
              "disabled";
  }

  double forward_rel_time = veh_rel_time + planning_cycle_time;  //往前推0.1s

  size_t forward_time_index =     //查找到在上一帧轨迹中从当前时间向后推0.1s后的轨迹index
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  ADEBUG << "Position matched index:\t" << position_matched_index;   //求两种匹配的最小值
  ADEBUG << "Time matched index:\t" << time_matched_index;

  auto matched_index = std::min(time_matched_index, position_matched_index);

  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->begin() +
          std::max(0, static_cast<int>(matched_index - preserved_points_num)),  //往后多取出20个点
      prev_trajectory->begin() + forward_time_index + 1);   //往前推forward_time_index个点，把这个点作为规划的起点
  ADEBUG << "stitching_trajectory size: " << stitching_trajectory.size();

  const double zero_s = stitching_trajectory.back().path_point().s();
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {   //所有的轨迹点必须是有效的
      *replan_reason = "replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -  //注意计算相对时间的方式，因为是从上一帧拿的数据，那么每个点的相对时间就为上一帧对应点的绝对时间减去当前时间
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);  //拼接的轨迹最后一个点作为规划的起点，那么之前的这段轨迹上的s当然就是为负数
  }
  return stitching_trajectory;
}

std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y, const TrajectoryPoint& p) {
  Vec2d v(x - p.path_point().x(), y - p.path_point().y());
  Vec2d n(std::cos(p.path_point().theta()), std::sin(p.path_point().theta()));

  std::pair<double, double> frenet_sd;
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();
  frenet_sd.second = v.CrossProd(n);
  return frenet_sd;
}

}  // namespace planning
}  // namespace apollo
