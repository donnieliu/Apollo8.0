/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.h"

#include <algorithm>
#include <memory>
#include <string>

#include "modules/planning/common/obstacle_blocking_analyzer.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

constexpr double kIntersectionClearanceDist = 20.0;
constexpr double kJunctionClearanceDist = 15.0;

PathLaneBorrowDecider::PathLaneBorrowDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {}
//借道决策器主要功能为判断当前车辆是否具备借道能力，具体实现在下面的process函数中实现，这个函数共有三部分：检查输入，如果路径服用则跳过借道决策，判断当前借道状态
Status PathLaneBorrowDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // skip path_lane_borrow_decider if reused path
  if (FLAGS_enable_skip_path_tasks && reference_line_info->path_reusable()) {  //如果path_reusable置位后，后续的task会跳过处理的过程，比如此处
    // for debug
    AINFO << "skip due to reusing path";
    return Status::OK();
  }

  // By default, don't borrow any lane.
  reference_line_info->set_is_path_lane_borrow(false);
  // Check if lane-borrowing is needed, if so, borrow lane.
  //检查是否需要借道，如果需要借道，将当前reference_line的借道属性置位为TRUE
  if (Decider::config_.path_lane_borrow_decider_config()
          .allow_lane_borrowing() &&
      IsNecessaryToBorrowLane(*frame, *reference_line_info)) {
    reference_line_info->set_is_path_lane_borrow(true);
  }
  return Status::OK();
}
//判断是否借道
//这个函数主要涉及一些rules，包括距离信号交叉口的距离，与障碍物的距离，是否是单行道，是否所在车道左右车道线是虚线等规则。主要有两个功能：
//（1）已处于借道场景下判断是否退出避让；
//（2）未处于借道场景下判断是否具备借道能力
bool PathLaneBorrowDecider::IsNecessaryToBorrowLane(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  //如果当前处于借道场景中
  if (mutable_path_decider_status->is_in_path_lane_borrow_scenario()) {
    // If originally borrowing neighbor lane:
    //able_to_use_self_lane_counter：如果直行，则一直向上增加不超过10，如果非直行，则清零；直行超过6帧取消换道场景
    if (mutable_path_decider_status->able_to_use_self_lane_counter() >= 6) { 
      // If have been able to use self-lane for some time, then switch to
      // non-lane-borrowing.
      mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
      mutable_path_decider_status->clear_decided_side_pass_direction();
      AINFO << "Switch from LANE-BORROW path to SELF-LANE path.";
    }
   //如果当前没有处于借道场景中
  } else {
    // If originally not borrowing neighbor lane:
    ADEBUG << "Blocking obstacle ID["
           << mutable_path_decider_status->front_static_obstacle_id() << "]";
    // ADC requirements check for lane-borrowing:
    //当下面这些条件必须全部满足，才能借道
    //只有一条参考线，才能借道；
    //起点速度小于最大借道允许速度
    //阻塞障碍物必须远离路口
    //阻塞障碍物会一直存在
    //阻塞障碍物与终点位置满足要求
    //为可侧面通过的障碍物
    if (!HasSingleReferenceLine(frame)) {
      return false;
    }
    if (!IsWithinSidePassingSpeedADC(frame)) {
      return false;
    }

    // Obstacle condition check for lane-borrowing:
    if (!IsBlockingObstacleFarFromIntersection(reference_line_info)) {
      return false;
    }
    if (!IsLongTermBlockingObstacle()) {
      return false;
    }
    if (!IsBlockingObstacleWithinDestination(reference_line_info)) {
      return false;
    }
    if (!IsSidePassableObstacle(reference_line_info)) {
      return false;
    }

    // switch to lane-borrowing
    // set side-pass direction
    //在无避让方向时重新计算避让方向，若左、右借道空间均不满足则不借道，is_in_path_lane_borrow_scenario_标志位为false
    //左借道条件满足左借道，右借道条件满足右借道，is_in_path_lane_borrow_scenario_为TRUE
    const auto& path_decider_status =
        injector_->planning_context()->planning_status().path_decider();
    if (path_decider_status.decided_side_pass_direction().empty()) {
      // first time init decided_side_pass_direction
      bool left_borrowable;
      bool right_borrowable;
      CheckLaneBorrow(reference_line_info, &left_borrowable, &right_borrowable);
      if (!left_borrowable && !right_borrowable) {
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
        return false;
      } else {
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
        if (left_borrowable) {
          mutable_path_decider_status->add_decided_side_pass_direction(
              PathDeciderStatus::LEFT_BORROW);
        }
        if (right_borrowable) {
          mutable_path_decider_status->add_decided_side_pass_direction(
              PathDeciderStatus::RIGHT_BORROW);
        }
      }
    }

    AINFO << "Switch from SELF-LANE path to LANE-BORROW path.";
  }
  return mutable_path_decider_status->is_in_path_lane_borrow_scenario();
}

// This function is to prevent lane-borrowing during lane-changing.
// TODO(jiacheng): depending on our needs, may allow lane-borrowing during
//                 lane-change.
bool PathLaneBorrowDecider::HasSingleReferenceLine(const Frame& frame) {
  return frame.reference_line_info().size() == 1;
}

bool PathLaneBorrowDecider::IsWithinSidePassingSpeedADC(const Frame& frame) {
  return frame.PlanningStartPoint().v() < FLAGS_lane_borrow_max_speed; //5m/s
}

bool PathLaneBorrowDecider::IsLongTermBlockingObstacle() {
  if (injector_->planning_context()
          ->planning_status()
          .path_decider()
          .front_static_obstacle_cycle_counter() >=
      FLAGS_long_term_blocking_obstacle_cycle_threshold) { //3
    ADEBUG << "The blocking obstacle is long-term existing.";
    return true;
  } else {
    ADEBUG << "The blocking obstacle is not long-term existing.";
    return false;
  }
}

bool PathLaneBorrowDecider::IsBlockingObstacleWithinDestination(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().start_s();
  double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  ADEBUG << "ADC is at s = " << adc_end_s;
  ADEBUG << "Destination is at s = "
         << reference_line_info.SDistanceToDestination() + adc_end_s;
  if (blocking_obstacle_s - adc_end_s >
      reference_line_info.SDistanceToDestination()) {  //障碍物在终点后边，那么则不算blockingwithindestination
    return false;
  }
  return true;
}

bool PathLaneBorrowDecider::IsBlockingObstacleFarFromIntersection(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();  //只有pathassessment_dedicer中有赋值
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return true;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return true;
  }

  // Get blocking obstacle's s.
  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().end_s(); //注意这里是end_s
  ADEBUG << "Blocking obstacle is at s = " << blocking_obstacle_s;
  // Get intersection's s and compare with threshold.
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  for (const auto& overlap : first_encountered_overlaps) {
    ADEBUG << overlap.first << ", " << overlap.second.DebugString();
    // if (// overlap.first != ReferenceLineInfo::CLEAR_AREA &&
    // overlap.first != ReferenceLineInfo::CROSSWALK &&
    // overlap.first != ReferenceLineInfo::PNC_JUNCTION &&
    if (overlap.first != ReferenceLineInfo::SIGNAL &&   //其他非SIGNAL and STOP_SIGN，则略过
        overlap.first != ReferenceLineInfo::STOP_SIGN) {
      continue;
    }

    auto distance = overlap.second.start_s - blocking_obstacle_s;
    if (overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN) {
      if (distance < kIntersectionClearanceDist) {  //20；距离交叉口过小了，那么也不允许借道
        ADEBUG << "Too close to signal intersection (" << distance
               << "m); don't SIDE_PASS.";
        return false;
      }
    } else {
      if (distance < kJunctionClearanceDist) {  //15；如果是非SIGNAL或非STOP_SIGN，这个阈值变小了
        ADEBUG << "Too close to overlap_type[" << overlap.first << "] ("
               << distance << "m); don't SIDE_PASS";
        return false;
      }
    }
  }

  return true;
}

bool PathLaneBorrowDecider::IsSidePassableObstacle(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    ADEBUG << "There is no blocking obstacle.";
    return false;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    ADEBUG << "Blocking obstacle is no longer there.";
    return false;
  }

  return IsNonmovableObstacle(reference_line_info, *blocking_obstacle);
}
//主要根据前方道路的线型判断是否可以借道；在此函数中2m间隔一个点遍历车前100m参考线或全部参考线，如果车道线类型为黄实线，白实线则不借道
void PathLaneBorrowDecider::CheckLaneBorrow(
    const ReferenceLineInfo& reference_line_info,
    bool* left_neighbor_lane_borrowable, bool* right_neighbor_lane_borrowable) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  *left_neighbor_lane_borrowable = true;
  *right_neighbor_lane_borrowable = true;

  static constexpr double kLookforwardDistance = 100.0;
  double check_s = reference_line_info.AdcSlBoundary().end_s();
  const double lookforward_distance =
      std::min(check_s + kLookforwardDistance, reference_line.Length());
  while (check_s < lookforward_distance) {
    auto ref_point = reference_line.GetNearestReferencePoint(check_s);
    if (ref_point.lane_waypoints().empty()) {
      *left_neighbor_lane_borrowable = false;
      *right_neighbor_lane_borrowable = false;
      return;
    }

    const auto waypoint = ref_point.lane_waypoints().front();
    hdmap::LaneBoundaryType::Type lane_boundary_type =
        hdmap::LaneBoundaryType::UNKNOWN;

    if (*left_neighbor_lane_borrowable) {
      lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
      if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
          lane_boundary_type == hdmap::LaneBoundaryType::DOUBLE_YELLOW ||
          lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
        *left_neighbor_lane_borrowable = false;
      }
      ADEBUG << "s[" << check_s << "] left_lane_boundary_type["
             << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
    }
    if (*right_neighbor_lane_borrowable) {
      lane_boundary_type = hdmap::RightBoundaryType(waypoint);
      if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
          lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
        *right_neighbor_lane_borrowable = false;
      }
      ADEBUG << "s[" << check_s << "] right_neighbor_lane_borrowable["
             << LaneBoundaryType_Type_Name(lane_boundary_type) << "]";
    }
    check_s += 2.0;
  }
}

}  // namespace planning
}  // namespace apollo
