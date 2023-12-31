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

#include "modules/planning/tasks/deciders/path_decider/path_decider.h"

#include <memory>

#include "modules/common_msgs/planning_msgs/decision.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

PathDecider::PathDecider(const TaskConfig &config,
                         const std::shared_ptr<DependencyInjector> &injector)
    : Task(config, injector) {}

Status PathDecider::Execute(Frame *frame,
                            ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Process(reference_line_info, reference_line_info->path_data(),
                 reference_line_info->path_decision());
}

Status PathDecider::Process(const ReferenceLineInfo *reference_line_info,
                            const PathData &path_data,
                            PathDecision *const path_decision) {
  // skip path_decider if reused path
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {
    return Status::OK();
  }

  std::string blocking_obstacle_id;
  if (reference_line_info->GetBlockingObstacle() != nullptr) {
    blocking_obstacle_id = reference_line_info->GetBlockingObstacle()->Id();
  }
  if (!MakeObjectDecision(path_data, blocking_obstacle_id, path_decision)) {
    const std::string msg = "Failed to make decision based on tunnel";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}
// MakeStaticObstacleDecision(...)函数作用：对静态障碍物做决策，若障碍物边界距离自车未来边界最近的横向距离大于3m就横向Ignore，
// 若<0.15m就做纵向stop决策，若[0.15,3.0]m时就做nudge决策，nudge决策也分为车道内向左绕行，车道内向右绕行。
bool PathDecider::MakeObjectDecision(const PathData &path_data,
                                     const std::string &blocking_obstacle_id,
                                     PathDecision *const path_decision) {
  //path decider的功能在MakeStaticObstacleDecision中
  if (!MakeStaticObstacleDecision(path_data, blocking_obstacle_id,
                                  path_decision)) {
    AERROR << "Failed to make decisions for static obstacles";
    return false;
  }
  return true;
}

// TODO(jiacheng): eventually this entire "path_decider" should be retired.
// Before it gets retired, its logics are slightly modified so that everything
// still works well for now.
bool PathDecider::MakeStaticObstacleDecision(
    const PathData &path_data, const std::string &blocking_obstacle_id,
    PathDecision *const path_decision) {
  // Sanity checks and get important values.
  ACHECK(path_decision);
  const auto &frenet_path = path_data.frenet_frame_path();
  if (frenet_path.empty()) {
    AERROR << "Path is empty.";
    return false;
  }
  const double half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;
  const double lateral_radius = half_width + FLAGS_lateral_ignore_buffer; //3m

  // Go through every obstacle and make decisions.
  //遍历每个障碍物，做决策
  for (const auto *obstacle : path_decision->obstacles().Items()) {
    const std::string &obstacle_id = obstacle->Id();
    const std::string obstacle_type_name =
        PerceptionObstacle_Type_Name(obstacle->Perception().type());
    ADEBUG << "obstacle_id[<< " << obstacle_id << "] type["
           << obstacle_type_name << "]";
    //如果障碍物不是静态的或者是virtual的，就跳过
    if (!obstacle->IsStatic() || obstacle->IsVirtual()) {
      continue;
    }
    // - skip decision making for obstacles with IGNORE/STOP decisions already.
    //如果障碍物已经有了ignore/stop决策，就跳过
    if (obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_ignore() &&
        obstacle->HasLateralDecision() &&
        obstacle->LateralDecision().has_ignore()) {
      continue;
    }
    if (obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_stop()) {
      // STOP decision
      continue;
    }
    // - add STOP decision for blocking obstacles.
    //如果障碍物挡住了路径，加stop决策
    //根据blocking_obstacle_id判断是否挡住路径，这个变量在前面的task中会进行计算
    if (obstacle->Id() == blocking_obstacle_id && //加入非借道场景是什么意思？？借道就没有block吗？
        !injector_->planning_context()
             ->planning_status()
             .path_decider()
             .is_in_path_lane_borrow_scenario()) {
      // Add stop decision
      ADEBUG << "Blocking obstacle = " << blocking_obstacle_id;
      ObjectDecisionType object_decision;
      *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);
      path_decision->AddLongitudinalDecision("PathDecider/blocking_obstacle",
                                             obstacle->Id(), object_decision);
      continue;
    }
    // - skip decision making for clear-zone obstacles.
    //如果是clear-zone跳过
    if (obstacle->reference_line_st_boundary().boundary_type() ==
        STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    // 0. IGNORE by default and if obstacle is not in path s at all.
    //如果障碍物不在路径上，跳过
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();
    const auto &sl_boundary = obstacle->PerceptionSLBoundary();
    if (sl_boundary.end_s() < frenet_path.front().s() ||
        sl_boundary.start_s() > frenet_path.back().s()) {
      path_decision->AddLongitudinalDecision("PathDecider/not-in-s",
                                             obstacle->Id(), object_decision);
      path_decision->AddLateralDecision("PathDecider/not-in-s", obstacle->Id(),
                                        object_decision);
      continue;
    }

    const auto frenet_point = frenet_path.GetNearestPoint(sl_boundary); //frenet_point就是在自车path路径上找到距离障碍物SLBoundary最近的点
    const double curr_l = frenet_point.l();
    double min_nudge_l =
        half_width +
        config_.path_decider_config().static_obstacle_buffer() / 2.0;  //half + 0.3/2
    //nudge判断，如果距离静态障碍物距离太远，则忽略。
    //如果静态障碍物距离车道中心太近，则停止
    //如果横向方向很近，则避开
    if (curr_l - lateral_radius > sl_boundary.end_l() || //障碍物太远了，超过了考虑的范围
        curr_l + lateral_radius < sl_boundary.start_l()) {
      // 1. IGNORE if laterally too far away.
      path_decision->AddLateralDecision("PathDecider/not-in-l", obstacle->Id(),
                                        object_decision);
    } else if (sl_boundary.end_l() >= curr_l - min_nudge_l &&
               sl_boundary.start_l() <= curr_l + min_nudge_l) {
      // 2. STOP if laterally too overlapping.
      *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);

      if (path_decision->MergeWithMainStop(
              object_decision.stop(), obstacle->Id(),
              reference_line_info_->reference_line(),
              reference_line_info_->AdcSlBoundary())) {
        path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                               obstacle->Id(), object_decision);
      } else {
        ObjectDecisionType object_decision;
        object_decision.mutable_ignore();
        path_decision->AddLongitudinalDecision("PathDecider/not-nearest-stop",
                                               obstacle->Id(), object_decision);
      }
    } else {
      // 3. NUDGE if laterally very close.
      if (sl_boundary.end_l() < curr_l - min_nudge_l) {  // &&
        // sl_boundary.end_l() > curr_l - min_nudge_l - 0.3) {
        // LEFT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
        object_nudge_ptr->set_distance_l(
            config_.path_decider_config().static_obstacle_buffer());
        path_decision->AddLateralDecision("PathDecider/left-nudge",
                                          obstacle->Id(), object_decision);
      } else if (sl_boundary.start_l() > curr_l + min_nudge_l) {  // &&
        // sl_boundary.start_l() < curr_l + min_nudge_l + 0.3) {
        // RIGHT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
        object_nudge_ptr->set_distance_l(
            -config_.path_decider_config().static_obstacle_buffer());
        path_decision->AddLateralDecision("PathDecider/right-nudge",
                                          obstacle->Id(), object_decision);
      }
    }
  }

  return true;
}

ObjectStop PathDecider::GenerateObjectStopDecision(
    const Obstacle &obstacle) const {
  ObjectStop object_stop;

  double stop_distance = obstacle.MinRadiusStopDistance(
      VehicleConfigHelper::GetConfig().vehicle_param());
  object_stop.set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
  object_stop.set_distance_s(-stop_distance); //注意这个地方stop_distance取负，说的是相对于障碍物，停止点在障碍物的后方

  const double stop_ref_s =
      obstacle.PerceptionSLBoundary().start_s() - stop_distance;
  const auto stop_ref_point =
      reference_line_info_->reference_line().GetReferencePoint(stop_ref_s); //这个地方取的参考线上点，为什么不是path上，可能参考线比较长比path，切在中心，适合选取停车点
  object_stop.mutable_stop_point()->set_x(stop_ref_point.x());
  object_stop.mutable_stop_point()->set_y(stop_ref_point.y());
  object_stop.set_stop_heading(stop_ref_point.heading());
  return object_stop;
}

}  // namespace planning
}  // namespace apollo
