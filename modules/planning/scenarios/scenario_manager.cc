/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/scenario_manager.h"

#include <string>
#include <vector>
#include <algorithm>

#include "modules/common_msgs/map_msgs/map_lane.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/util/util.h"
#include "modules/planning/scenarios/bare_intersection/unprotected/bare_intersection_unprotected_scenario.h"
#include "modules/planning/scenarios/emergency/emergency_pull_over/emergency_pull_over_scenario.h"
#include "modules/planning/scenarios/emergency/emergency_stop/emergency_stop_scenario.h"
#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"
#include "modules/planning/scenarios/learning_model/learning_model_sample_scenario.h"
#include "modules/planning/scenarios/park/pull_over/pull_over_scenario.h"
#include "modules/planning/scenarios/park/valet_parking/valet_parking_scenario.h"
#include "modules/planning/scenarios/park_and_go/park_and_go_scenario.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stop_sign_unprotected_scenario.h"
#include "modules/planning/scenarios/traffic_light/protected/traffic_light_protected_scenario.h"
#include "modules/planning/scenarios/traffic_light/unprotected_left_turn/traffic_light_unprotected_left_turn_scenario.h"
#include "modules/planning/scenarios/traffic_light/unprotected_right_turn/traffic_light_unprotected_right_turn_scenario.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/scenarios/yield_sign/yield_sign_scenario.h"

namespace apollo {
namespace planning {
namespace scenario {

using apollo::hdmap::HDMapUtil;
using apollo::hdmap::PathOverlap;

ScenarioManager::ScenarioManager(
    const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {}

bool ScenarioManager::Init(const PlanningConfig& planning_config) {
  planning_config_.CopyFrom(planning_config);
  RegisterScenarios();  //注册场景
  default_scenario_type_ = ScenarioType::LANE_FOLLOW;
  current_scenario_ = CreateScenario(default_scenario_type_);   //创建场景，默认场景为lane_following
  return true;
}

std::unique_ptr<Scenario> ScenarioManager::CreateScenario(
    ScenarioType scenario_type) {
  std::unique_ptr<Scenario> ptr;

  switch (scenario_type) {
    case ScenarioType::BARE_INTERSECTION_UNPROTECTED:
      ptr.reset(
          new scenario::bare_intersection::BareIntersectionUnprotectedScenario(
              config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioType::EMERGENCY_PULL_OVER:
      ptr.reset(new emergency_pull_over::EmergencyPullOverScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioType::EMERGENCY_STOP:
      ptr.reset(new emergency_stop::EmergencyStopScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioType::LANE_FOLLOW:
      ptr.reset(new lane_follow::LaneFollowScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioType::LEARNING_MODEL_SAMPLE:
      ptr.reset(new scenario::LearningModelSampleScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioType::PARK_AND_GO:
      ptr.reset(new scenario::park_and_go::ParkAndGoScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioType::PULL_OVER:
      ptr.reset(new scenario::pull_over::PullOverScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioType::STOP_SIGN_UNPROTECTED:
      ptr.reset(new scenario::stop_sign::StopSignUnprotectedScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioType::TRAFFIC_LIGHT_PROTECTED:
      ptr.reset(new scenario::traffic_light::TrafficLightProtectedScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
      ptr.reset(
          new scenario::traffic_light::TrafficLightUnprotectedLeftTurnScenario(
              config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
      ptr.reset(
          new scenario::traffic_light::TrafficLightUnprotectedRightTurnScenario(
              config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioType::VALET_PARKING:
      ptr.reset(new scenario::valet_parking::ValetParkingScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    case ScenarioType::YIELD_SIGN:
      ptr.reset(new scenario::yield_sign::YieldSignScenario(
          config_map_[scenario_type], &scenario_context_, injector_));
      break;
    default:
      return nullptr;
  }

  if (ptr != nullptr) {
    ptr->Init();
  }
  return ptr;
}

void ScenarioManager::RegisterScenarios() {
  // lane_follow
  //注册场景到config_map_中
  if (planning_config_.learning_mode() == PlanningConfig::HYBRID ||
      planning_config_.learning_mode() == PlanningConfig::HYBRID_TEST) {
    // HYBRID or HYBRID_TEST
    ACHECK(Scenario::LoadConfig(FLAGS_scenario_lane_follow_hybrid_config_file,
                                &config_map_[ScenarioType::LANE_FOLLOW]));
  } else {
    ACHECK(Scenario::LoadConfig(FLAGS_scenario_lane_follow_config_file,
                                &config_map_[ScenarioType::LANE_FOLLOW]));
  }

  // bare_intersection :没有stop sign和信号灯的路口
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_bare_intersection_unprotected_config_file,
      &config_map_[ScenarioType::BARE_INTERSECTION_UNPROTECTED]));

  // emergency_pull_over:紧急靠边停车
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_emergency_pull_over_config_file,  
                              &config_map_[ScenarioType::EMERGENCY_PULL_OVER]));

  // emergency_stop
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_emergency_stop_config_file,
                              &config_map_[ScenarioType::EMERGENCY_STOP]));

  // learning model
  ACHECK(
      Scenario::LoadConfig(FLAGS_scenario_learning_model_sample_config_file,
                           &config_map_[ScenarioType::LEARNING_MODEL_SAMPLE]));

 // park_and_go
  // 该场景的判断条件为车辆是否静止，并且距离终点10m以上，并且当前车辆已经off_lane或者不在城市道路上，
  // 在该场景下采用的是open_space相关的算法。个人感觉该场景在驶离目标车道并正常规划失败导致的停车时会触发，
  // 利用open_space方法使其重新回到正常道路上
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_park_and_go_config_file,
                              &config_map_[ScenarioType::PARK_AND_GO]));

  // pull_over
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_pull_over_config_file,
                              &config_map_[ScenarioType::PULL_OVER]));

  // stop_sign
  ACHECK(
      Scenario::LoadConfig(FLAGS_scenario_stop_sign_unprotected_config_file,
                           &config_map_[ScenarioType::STOP_SIGN_UNPROTECTED]));

  // traffic_light
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_traffic_light_protected_config_file,
      &config_map_[ScenarioType::TRAFFIC_LIGHT_PROTECTED]));
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_traffic_light_unprotected_left_turn_config_file,
      &config_map_[ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN]));
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_traffic_light_unprotected_right_turn_config_file,
      &config_map_[ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN]));

  // valet parking: 代客泊车
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_valet_parking_config_file,
                              &config_map_[ScenarioType::VALET_PARKING]));

  // yield_sign
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_yield_sign_config_file,
                              &config_map_[ScenarioType::YIELD_SIGN]));
}

ScenarioType ScenarioManager::SelectPullOverScenario(const Frame& frame) {
  const auto& scenario_config =
      config_map_[ScenarioType::PULL_OVER].pull_over_config();

  const auto& routing = frame.local_view().routing;
  const auto& routing_end = *(routing->routing_request().waypoint().rbegin());

  common::SLPoint dest_sl;
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  reference_line.XYToSL(routing_end.pose(), &dest_sl);
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s; //目的地跟前车脸的距离
  ADEBUG << "adc_distance_to_dest[" << adc_distance_to_dest
         << "] destination_s[" << dest_sl.s() << "] adc_front_edge_s["
         << adc_front_edge_s << "]";

  bool pull_over_scenario =
      (frame.reference_line_info().size() == 1 &&  // NO, while changing lane
       adc_distance_to_dest >=
           scenario_config.pull_over_min_distance_buffer() && //10米；adc_distance_to_dest：10到50之间
       adc_distance_to_dest <=
           scenario_config.start_pull_over_scenario_distance());  //50米

  // too close to destination + not found pull-over position
  if (pull_over_scenario) {
    const auto& pull_over_status =
        injector_->planning_context()->planning_status().pull_over();
    if (adc_distance_to_dest < scenario_config.max_distance_stop_search() && //25米
        !pull_over_status.has_position()) {
      pull_over_scenario = false;
    }
  }

  // check around junction //pullover的终点不能在交叉路口附近
  if (pull_over_scenario) {
    static constexpr double kDistanceToAvoidJunction = 8.0;  // meter
    for (const auto& overlap : first_encountered_overlap_map_) {
      if (overlap.first == ReferenceLineInfo::PNC_JUNCTION ||
          overlap.first == ReferenceLineInfo::SIGNAL ||
          overlap.first == ReferenceLineInfo::STOP_SIGN ||
          overlap.first == ReferenceLineInfo::YIELD_SIGN) {
        const double distance_to = overlap.second.start_s - dest_sl.s(); //overlap在停车点后边不远处
        const double distance_passed = dest_sl.s() - overlap.second.end_s; //overlap在停车点前边不远处
        if ((distance_to > 0.0 && distance_to < kDistanceToAvoidJunction) ||
            (distance_passed > 0.0 &&
             distance_passed < kDistanceToAvoidJunction)) {
          pull_over_scenario = false;
          break;
        }
      }
    }
  }

  // check rightmost driving lane along pull-over path //能查找到最右边车道的lane_type，并且该车道允许pullover
  if (pull_over_scenario) {
    double check_s = adc_front_edge_s;
    static constexpr double kDistanceUnit = 5.0;
    while (check_s < dest_sl.s()) {
      check_s += kDistanceUnit;

      std::vector<hdmap::LaneInfoConstPtr> lanes;
      reference_line.GetLaneFromS(check_s, &lanes);
      if (lanes.empty()) {
        ADEBUG << "check_s[" << check_s << "] can't find a lane";
        continue;
      }
      const hdmap::LaneInfoConstPtr lane = lanes[0]; //这个lane[0]应该是当前车道的意思
      const std::string lane_id = lane->lane().id().id();
      ADEBUG << "check_s[" << check_s << "] lane[" << lane_id << "]";

      // check neighbor lanes type: NONE/CITY_DRIVING/BIKING/SIDEWALK/PARKING
      //能查找到最右边车道的lane_type，并且该车道允许pullover
      bool rightmost_driving_lane = true;
      for (const auto& neighbor_lane_id :
           lane->lane().right_neighbor_forward_lane_id()) {
        const auto hdmap_ptr = HDMapUtil::BaseMapPtr();
        CHECK_NOTNULL(hdmap_ptr);
        const auto neighbor_lane = hdmap_ptr->GetLaneById(neighbor_lane_id);
        if (neighbor_lane == nullptr) {
          ADEBUG << "Failed to find neighbor lane[" << neighbor_lane_id.id()
                 << "]";
          continue;
        }
        const auto& lane_type = neighbor_lane->lane().type();
        if (lane_type == hdmap::Lane::CITY_DRIVING) {
          ADEBUG << "lane[" << lane_id << "]'s right neighbor forward lane["
                 << neighbor_lane_id.id() << "] type["
                 << Lane_LaneType_Name(lane_type) << "] can't pull over";
          rightmost_driving_lane = false; //若找到右边车道，最右侧车道是机动车道，那么也不允许进入pullover场景
          break;
        }
      }
      if (!rightmost_driving_lane) { //如果最右侧车道不允许pullover（比如机动车道），那么也不会进入pullover的场景
        pull_over_scenario = false;
        break;
      }
    }
  }

  switch (current_scenario_->scenario_type()) {
    case ScenarioType::LANE_FOLLOW:
      if (pull_over_scenario) {
        return ScenarioType::PULL_OVER;
      }
      break;
    case ScenarioType::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioType::EMERGENCY_PULL_OVER:
    case ScenarioType::PARK_AND_GO:
    case ScenarioType::PULL_OVER:
    case ScenarioType::STOP_SIGN_PROTECTED:
    case ScenarioType::STOP_SIGN_UNPROTECTED:
    case ScenarioType::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioType::VALET_PARKING:
    case ScenarioType::YIELD_SIGN:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioType ScenarioManager::SelectPadMsgScenario(const Frame& frame) {   //判断驾驶员意图，其实就是除了一些紧急情况外，都返回lanefollwing
  const auto& pad_msg_driving_action = frame.GetPadMsgDrivingAction();

  switch (pad_msg_driving_action) {
    case PadMessage::PULL_OVER:
      if (FLAGS_enable_scenario_emergency_pull_over) {  //true
        return ScenarioType::EMERGENCY_PULL_OVER;
      }
      break;
    case PadMessage::STOP:
      if (FLAGS_enable_scenario_emergency_stop) {  //true
        return ScenarioType::EMERGENCY_STOP;
      }
      break;
    case PadMessage::RESUME_CRUISE:
      if (current_scenario_->scenario_type() ==
              ScenarioType::EMERGENCY_PULL_OVER ||
          current_scenario_->scenario_type() == ScenarioType::EMERGENCY_STOP) {
        return ScenarioType::PARK_AND_GO; 
      }
      break;
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioType ScenarioManager::SelectInterceptionScenario(const Frame& frame) {
  ScenarioType scenario_type = default_scenario_type_;

  hdmap::PathOverlap* traffic_sign_overlap = nullptr;
  hdmap::PathOverlap* pnc_junction_overlap = nullptr;
  ReferenceLineInfo::OverlapType overlap_type;

  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  // note: first_encountered_overlaps already sorted
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::SIGNAL || //信号灯，停止线，让行标志
        overlap.first == ReferenceLineInfo::STOP_SIGN ||
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      overlap_type = overlap.first;
      traffic_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
      break;
    } else if (overlap.first == ReferenceLineInfo::PNC_JUNCTION) { //岔路口
      pnc_junction_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
    }
  }

  // pick a closer one between consecutive bare_intersection and traffic_sign
  if (traffic_sign_overlap && pnc_junction_overlap) {
    static constexpr double kJunctionDelta = 10.0;
    double s_diff = std::fabs(traffic_sign_overlap->start_s -
                              pnc_junction_overlap->start_s);
    if (s_diff >= kJunctionDelta) {
      if (pnc_junction_overlap->start_s > traffic_sign_overlap->start_s) {
        pnc_junction_overlap = nullptr;
      } else {
        traffic_sign_overlap = nullptr;
      }
    }
  }

  if (traffic_sign_overlap) {
    switch (overlap_type) {
      case ReferenceLineInfo::STOP_SIGN:
        if (FLAGS_enable_scenario_stop_sign) { //true
          scenario_type = SelectStopSignScenario(frame, *traffic_sign_overlap);
        }
        break;
      case ReferenceLineInfo::SIGNAL:
        if (FLAGS_enable_scenario_traffic_light) {  //true
          scenario_type =
              SelectTrafficLightScenario(frame, *traffic_sign_overlap);
        }
        break;
      case ReferenceLineInfo::YIELD_SIGN:
        if (FLAGS_enable_scenario_yield_sign) {  //true
          scenario_type = SelectYieldSignScenario(frame, *traffic_sign_overlap);
        }
        break;
      default:
        break;
    }
  } else if (pnc_junction_overlap) {
    // bare intersection
    if (FLAGS_enable_scenario_bare_intersection) {  //true
      scenario_type =
          SelectBareIntersectionScenario(frame, *pnc_junction_overlap);
    }
  }

  return scenario_type;
}

ScenarioType ScenarioManager::SelectStopSignScenario(
    const Frame& frame, const hdmap::PathOverlap& stop_sign_overlap) {
  const auto& scenario_config = config_map_[ScenarioType::STOP_SIGN_UNPROTECTED]
                                    .stop_sign_unprotected_config();

  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_stop_sign =
      stop_sign_overlap.start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_stop_sign[" << adc_distance_to_stop_sign
         << "] stop_sign[" << stop_sign_overlap.object_id
         << "] stop_sign_overlap_start_s[" << stop_sign_overlap.start_s << "]";

  const bool stop_sign_scenario =
      (adc_distance_to_stop_sign > 0.0 &&
       adc_distance_to_stop_sign <=
           scenario_config.start_stop_sign_scenario_distance());  //4m
  const bool stop_sign_all_way = false;  // TODO(all)

  switch (current_scenario_->scenario_type()) {
    case ScenarioType::LANE_FOLLOW:
    case ScenarioType::PARK_AND_GO:
    case ScenarioType::PULL_OVER:
      if (stop_sign_scenario) {
        return stop_sign_all_way ? ScenarioType::STOP_SIGN_PROTECTED
                                 : ScenarioType::STOP_SIGN_UNPROTECTED; //默认返回无保护停止标志场景
      }
      break;
    case ScenarioType::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioType::EMERGENCY_PULL_OVER:
    case ScenarioType::STOP_SIGN_PROTECTED:
    case ScenarioType::STOP_SIGN_UNPROTECTED:
    case ScenarioType::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioType::YIELD_SIGN:
    case ScenarioType::VALET_PARKING:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type(); //若没有完成当前场景，则返回当前场景
      }
      break;
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioType ScenarioManager::SelectTrafficLightScenario(
    const Frame& frame, const hdmap::PathOverlap& traffic_light_overlap) {
  // some scenario may need start sooner than the others
  const double start_check_distance =
      std::max({config_map_[ScenarioType::TRAFFIC_LIGHT_PROTECTED]
                    .traffic_light_protected_config()
                    .start_traffic_light_scenario_distance(),  //5m
                config_map_[ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN]
                    .traffic_light_unprotected_left_turn_config()  //30米
                    .start_traffic_light_scenario_distance(),
                config_map_[ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN]
                    .traffic_light_unprotected_right_turn_config()
                    .start_traffic_light_scenario_distance()}); //5米

  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  // find all the traffic light belong to
  // the same group as first encountered traffic light
  std::vector<hdmap::PathOverlap> next_traffic_lights;
  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps(); //所有的signal overlap
  for (const auto& overlap : traffic_light_overlaps) {
    const double dist = overlap.start_s - traffic_light_overlap.start_s; //traffic_light_overlap:first encounter signal overlap
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) { //2米范围内的signal overlap都放到next_traffic_lights，什么情况下signal会在两米范围内有多个？？
      next_traffic_lights.push_back(overlap);
    }
  }

  bool traffic_light_scenario = false;
  bool red_light = false;

  // note: need iterate all lights to check no RED/YELLOW/UNKNOWN
  for (const auto& traffic_light_overlap : next_traffic_lights) {
    const double adc_distance_to_traffic_light =
        traffic_light_overlap.start_s - adc_front_edge_s;
    ADEBUG << "traffic_light[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s
           << "] adc_distance_to_traffic_light["
           << adc_distance_to_traffic_light << "]";

    // enter traffic-light scenarios: based on distance only
    if (adc_distance_to_traffic_light <= 0.0 ||
        adc_distance_to_traffic_light > start_check_distance) { //不同的红绿灯检测距离不同，默认配置30米内就检测了，过了这个范围认为已经检测完成
      continue;
    }

    traffic_light_scenario = true;

    const auto& signal_color =
        frame.GetSignal(traffic_light_overlap.object_id).color();
    ADEBUG << "traffic_light_id[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s << "] color["
           << signal_color << "]";

    if (signal_color != perception::TrafficLight::GREEN) { //只要不是绿灯，就认为是红灯。
      red_light = true;
      break;
    }
  }

  bool traffic_light_protected_scenario = false;
  bool traffic_light_unprotected_left_turn_scenario = false;
  bool traffic_light_unprotected_right_turn_scenario = false;
  if (traffic_light_scenario) {
    const auto& turn_type =  //类型包含左转，右转，掉头以及不需要转弯类型
        reference_line_info.GetPathTurnType(traffic_light_overlap.start_s);
    const bool right_turn = (turn_type == hdmap::Lane::RIGHT_TURN);
    const bool left_turn = (turn_type == hdmap::Lane::LEFT_TURN);
    const double adc_distance_to_traffic_light =
        traffic_light_overlap.start_s - adc_front_edge_s;

    if (right_turn && red_light) {
      // check TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN
      const auto& scenario_config =
          config_map_[ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN]  
              .traffic_light_unprotected_right_turn_config(); 
      if (adc_distance_to_traffic_light <
          scenario_config.start_traffic_light_scenario_distance()) {  //5m
        traffic_light_unprotected_right_turn_scenario = true;
      }
    } else if (left_turn) {
      // check TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN
      const auto& scenario_config =
          config_map_[ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN]
              .traffic_light_unprotected_left_turn_config();
      if (adc_distance_to_traffic_light <
          scenario_config.start_traffic_light_scenario_distance()) { //30米
        traffic_light_unprotected_left_turn_scenario = true;
      }
    } else {
      // check TRAFFIC_LIGHT_PROTECTED
      const auto& scenario_config =
          config_map_[ScenarioType::TRAFFIC_LIGHT_PROTECTED]
              .traffic_light_protected_config();
      if (adc_distance_to_traffic_light <
          scenario_config.start_traffic_light_scenario_distance()) {  //5米
        traffic_light_protected_scenario = true;
      }
    }
  }

  switch (current_scenario_->scenario_type()) {
    case ScenarioType::LANE_FOLLOW:
    case ScenarioType::PARK_AND_GO:
    case ScenarioType::PULL_OVER:
      if (traffic_light_unprotected_left_turn_scenario) {
        return ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN;
      } else if (traffic_light_unprotected_right_turn_scenario) {
        return ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN;
      } else if (traffic_light_protected_scenario) {
        return ScenarioType::TRAFFIC_LIGHT_PROTECTED;
      }
      break;
    case ScenarioType::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioType::EMERGENCY_PULL_OVER:
    case ScenarioType::STOP_SIGN_PROTECTED:
    case ScenarioType::STOP_SIGN_UNPROTECTED:
    case ScenarioType::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioType::YIELD_SIGN:
    case ScenarioType::VALET_PARKING:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;

    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioType ScenarioManager::SelectYieldSignScenario(
    const Frame& frame, const hdmap::PathOverlap& yield_sign_overlap) {
  const auto& scenario_config =
      config_map_[ScenarioType::YIELD_SIGN].yield_sign_config();

  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_yield_sign =
      yield_sign_overlap.start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_yield_sign[" << adc_distance_to_yield_sign
         << "] yield_sign[" << yield_sign_overlap.object_id
         << "] yield_sign_overlap_start_s[" << yield_sign_overlap.start_s
         << "]";

  const bool yield_sign_scenario =
      (adc_distance_to_yield_sign > 0.0 &&
       adc_distance_to_yield_sign <=
           scenario_config.start_yield_sign_scenario_distance());  //10米

  switch (current_scenario_->scenario_type()) {
    case ScenarioType::LANE_FOLLOW:
    case ScenarioType::PARK_AND_GO:
    case ScenarioType::PULL_OVER:
      if (yield_sign_scenario) {
        return ScenarioType::YIELD_SIGN;
      }
      break;
    case ScenarioType::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioType::EMERGENCY_PULL_OVER:
    case ScenarioType::STOP_SIGN_PROTECTED:
    case ScenarioType::STOP_SIGN_UNPROTECTED:
    case ScenarioType::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioType::YIELD_SIGN:
    case ScenarioType::VALET_PARKING:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioType ScenarioManager::SelectBareIntersectionScenario(
    const Frame& frame, const hdmap::PathOverlap& pnc_junction_overlap) {
  const auto& reference_line_info = frame.reference_line_info().front();
  if (reference_line_info.GetIntersectionRightofWayStatus(  //没有转弯类型的话返回true，因此会直接维持lanefollow
          pnc_junction_overlap)) {
    return default_scenario_type_;
  }

  const auto& scenario_config =
      config_map_[ScenarioType::BARE_INTERSECTION_UNPROTECTED]
          .bare_intersection_unprotected_config();

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_pnc_junction =
      pnc_junction_overlap.start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_pnc_junction[" << adc_distance_to_pnc_junction
         << "] pnc_junction[" << pnc_junction_overlap.object_id
         << "] pnc_junction_overlap_start_s[" << pnc_junction_overlap.start_s
         << "]";

  const bool bare_junction_scenario =
      (adc_distance_to_pnc_junction > 0.0 &&
       adc_distance_to_pnc_junction <=
           scenario_config.start_bare_intersection_scenario_distance());  //25米

  switch (current_scenario_->scenario_type()) {
    case ScenarioType::LANE_FOLLOW:
    case ScenarioType::PARK_AND_GO:
    case ScenarioType::PULL_OVER:
      if (bare_junction_scenario) {
        return ScenarioType::BARE_INTERSECTION_UNPROTECTED;
      }
      break;
    case ScenarioType::BARE_INTERSECTION_UNPROTECTED:
    case ScenarioType::EMERGENCY_PULL_OVER:
    case ScenarioType::STOP_SIGN_PROTECTED:
    case ScenarioType::STOP_SIGN_UNPROTECTED:
    case ScenarioType::TRAFFIC_LIGHT_PROTECTED:
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
    case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
    case ScenarioType::YIELD_SIGN:
    case ScenarioType::VALET_PARKING:
      if (current_scenario_->GetStatus() !=
          Scenario::ScenarioStatus::STATUS_DONE) {
        return current_scenario_->scenario_type();
      }
      break;
    default:
      break;
  }

  return default_scenario_type_;
}

ScenarioType ScenarioManager::SelectValetParkingScenario(const Frame& frame) { //代客泊车场景
  const auto& scenario_config =
      config_map_[ScenarioType::VALET_PARKING].valet_parking_config();

  // TODO(All) trigger valet parking by route message definition as of now
  double parking_spot_range_to_start =
      scenario_config.parking_spot_range_to_start();  //20米
  if (scenario::valet_parking::ValetParkingScenario::IsTransferable(
          frame, parking_spot_range_to_start)) {
    return ScenarioType::VALET_PARKING;
  }

  return default_scenario_type_;
}

ScenarioType ScenarioManager::SelectParkAndGoScenario(const Frame& frame) {
  bool park_and_go = false;
  const auto& scenario_config =
      config_map_[ScenarioType::PARK_AND_GO].park_and_go_config();
  const auto vehicle_state_provider = injector_->vehicle_state();
  common::VehicleState vehicle_state = vehicle_state_provider->vehicle_state();
  auto adc_point = common::util::PointFactory::ToPointENU(vehicle_state);
  // TODO(SHU) might consider gear == GEAR_PARKING
  double adc_speed = vehicle_state_provider->linear_velocity();
  double s = 0.0;
  double l = 0.0;
  const double max_abs_speed_when_stopped =
      common::VehicleConfigHelper::Instance()
          ->GetConfig()
          .vehicle_param()
          .max_abs_speed_when_stopped();  //0.2

  hdmap::LaneInfoConstPtr lane;

  // check ego vehicle distance to destination
  const auto& routing = frame.local_view().routing;
  const auto& routing_end = *(routing->routing_request().waypoint().rbegin());
  common::SLPoint dest_sl;
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  reference_line.XYToSL(routing_end.pose(), &dest_sl);
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s;
  ADEBUG << "adc_distance_to_dest:" << adc_distance_to_dest;
  // if vehicle is static, far enough to destination and (off-lane or not on
  // city_driving lane)
  if (std::fabs(adc_speed) < max_abs_speed_when_stopped &&  //速度小于0.2
      adc_distance_to_dest > scenario_config.min_dist_to_dest() &&  //10米
      (HDMapUtil::BaseMap().GetNearestLaneWithHeading(
           adc_point, 2.0, vehicle_state.heading(), M_PI / 3.0, &lane, &s, //最近的lane存在
           &l) != 0 ||
       lane->lane().type() != hdmap::Lane::CITY_DRIVING)) {  //必须起步在非机动车道
    park_and_go = true;
  }

  if (park_and_go) {
    return ScenarioType::PARK_AND_GO;
  }

  return default_scenario_type_;
}

void ScenarioManager::Observe(const Frame& frame) {
  // init first_encountered_overlap_map_
  first_encountered_overlap_map_.clear();
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::PNC_JUNCTION ||
        overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN ||
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      first_encountered_overlap_map_[overlap.first] = overlap.second;
    }
  }
}
 //用来决策当前处在什么场景，如果进入了新的场景，会创建一个新的对象来进行之后的规划逻辑
 //场景决策逻辑在这个函数中，会根据配置选择基于规则还是基于学习的决策方法
void ScenarioManager::Update(const common::TrajectoryPoint& ego_point, 
                             const Frame& frame) {  
  ACHECK(!frame.reference_line_info().empty());

  Observe(frame);

  ScenarioDispatch(frame);
}

void ScenarioManager::ScenarioDispatch(const Frame& frame) {
  ACHECK(!frame.reference_line_info().empty());
  ScenarioType scenario_type;

  int history_points_len = 0;
  if (injector_->learning_based_data() &&
      injector_->learning_based_data()->GetLatestLearningDataFrame()) {
    history_points_len = injector_->learning_based_data()
                             ->GetLatestLearningDataFrame()
                             ->adc_trajectory_point_size();
  }
  if ((planning_config_.learning_mode() == PlanningConfig::E2E ||
       planning_config_.learning_mode() == PlanningConfig::E2E_TEST) &&
      history_points_len >= FLAGS_min_past_history_points_len) { //0
    scenario_type = ScenarioDispatchLearning(); 
  } else {
    scenario_type = ScenarioDispatchNonLearning(frame);
  }

  ADEBUG << "select scenario: " << ScenarioType_Name(scenario_type);

  // update PlanningContext
  UpdatePlanningContext(frame, scenario_type);

  if (current_scenario_->scenario_type() != scenario_type) {
    current_scenario_ = CreateScenario(scenario_type);
  }
}
//ScenarioDispatchNonLearning函数默认从lanefollow场景开始判断，首先根据驾驶员的意图来安排场景，如果不是默认的lanefollow场景，直接输出当前场景；如果
//是lanefollow场景，会依次判断是否属于别的场景；即剩余场景之间的跳转必须经过lanefollow这个场景。
ScenarioType ScenarioManager::ScenarioDispatchLearning() {
  ////////////////////////////////////////
  // learning model scenario
  ScenarioType scenario_type = ScenarioType::LEARNING_MODEL_SAMPLE;
  return scenario_type;
}

ScenarioType ScenarioManager::ScenarioDispatchNonLearning(const Frame& frame) {  //在场景判断时首先调用这个函数，根据驾驶员意图来安排场景
  ////////////////////////////////////////
  // default: LANE_FOLLOW
  ScenarioType scenario_type = default_scenario_type_;

  ////////////////////////////////////////
  // Pad Msg scenario
  scenario_type = SelectPadMsgScenario(frame);   //在场景判断时首先调用这个函数，根据驾驶员意图来安排场景
  //  如果pad没有场景输入，则根据当前场景进行场景跳转
  if (scenario_type == default_scenario_type_) {
    // check current_scenario (not switchable)
    //如果是LANE_FOLLOW或者是PULL_OVER，则保持当前场景
    switch (current_scenario_->scenario_type()) {
      case ScenarioType::LANE_FOLLOW:
      case ScenarioType::PULL_OVER:
        break;
      //如果是BARE_INTERSECTION_UNPROTECTED~YIELD_SIGN这些场景，则每周期要检查是否达到目的点附近，更新场景位LANE_FOLLOW,否则不改变场景
      case ScenarioType::BARE_INTERSECTION_UNPROTECTED:
      case ScenarioType::EMERGENCY_PULL_OVER:
      case ScenarioType::PARK_AND_GO:
      case ScenarioType::STOP_SIGN_PROTECTED:
      case ScenarioType::STOP_SIGN_UNPROTECTED:
      case ScenarioType::TRAFFIC_LIGHT_PROTECTED:
      case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN:
      case ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN:
      case ScenarioType::VALET_PARKING:
      //如果是YIELD_SIGN，则检查该场景是否完成。如果没有完成则保持让行场景
      case ScenarioType::YIELD_SIGN:
        // must continue until finish
        if (current_scenario_->GetStatus() !=
            Scenario::ScenarioStatus::STATUS_DONE) {
          scenario_type = current_scenario_->scenario_type();
        }
        break;
      default:
        break;
    }
  }

  ////////////////////////////////////////
  // ParkAndGo / starting scenario
  if (scenario_type == default_scenario_type_) {
    if (FLAGS_enable_scenario_park_and_go) {  //TRUE
      scenario_type = SelectParkAndGoScenario(frame);
    }
  }

  ////////////////////////////////////////
  // intersection scenarios
  //交叉路口场景
  if (scenario_type == default_scenario_type_) {
    scenario_type = SelectInterceptionScenario(frame);
  }

  ////////////////////////////////////////
  // pull-over scenario
  if (scenario_type == default_scenario_type_) {
    if (FLAGS_enable_scenario_pull_over) {
      scenario_type = SelectPullOverScenario(frame);
    }
  }

  ////////////////////////////////////////
  // VALET_PARKING scenario
  if (scenario_type == default_scenario_type_) {
    scenario_type = SelectValetParkingScenario(frame);  //代客泊车场景
  }

  return scenario_type;
}

bool ScenarioManager::IsBareIntersectionScenario(
    const ScenarioType& scenario_type) {
  return (scenario_type == ScenarioType::BARE_INTERSECTION_UNPROTECTED);
}

bool ScenarioManager::IsStopSignScenario(const ScenarioType& scenario_type) {
  return (scenario_type == ScenarioType::STOP_SIGN_PROTECTED ||
          scenario_type == ScenarioType::STOP_SIGN_UNPROTECTED);
}

bool ScenarioManager::IsTrafficLightScenario(
    const ScenarioType& scenario_type) {
  return (scenario_type == ScenarioType::TRAFFIC_LIGHT_PROTECTED ||
          scenario_type == ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN ||
          scenario_type == ScenarioType::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN);
}

bool ScenarioManager::IsYieldSignScenario(const ScenarioType& scenario_type) {
  return (scenario_type == ScenarioType::YIELD_SIGN);
}

void ScenarioManager::UpdatePlanningContext(const Frame& frame,
                                            const ScenarioType& scenario_type) {
  // BareIntersection scenario
  UpdatePlanningContextBareIntersectionScenario(frame, scenario_type);

  // EmergencyStop scenario
  UpdatePlanningContextEmergencyStopcenario(frame, scenario_type);

  // PullOver & EmergencyPullOver scenarios
  UpdatePlanningContextPullOverScenario(frame, scenario_type);

  // StopSign scenario
  UpdatePlanningContextStopSignScenario(frame, scenario_type);

  // TrafficLight scenario
  UpdatePlanningContextTrafficLightScenario(frame, scenario_type);

  // YieldSign scenario
  UpdatePlanningContextYieldSignScenario(frame, scenario_type);
}

// update: bare_intersection status in PlanningContext
void ScenarioManager::UpdatePlanningContextBareIntersectionScenario(
    const Frame& frame, const ScenarioType& scenario_type) {
  auto* bare_intersection = injector_->planning_context()
                                ->mutable_planning_status()
                                ->mutable_bare_intersection();

  if (!IsBareIntersectionScenario(scenario_type)) {
    bare_intersection->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // set to first_encountered pnc_junction
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::PNC_JUNCTION);
  if (map_itr != first_encountered_overlap_map_.end()) {
    bare_intersection->set_current_pnc_junction_overlap_id(
        map_itr->second.object_id);
    ADEBUG << "Update PlanningContext with first_encountered pnc_junction["
           << map_itr->second.object_id << "] start_s["
           << map_itr->second.start_s << "]";
  }
}

// update: emergency_stop status in PlanningContext
void ScenarioManager::UpdatePlanningContextEmergencyStopcenario(
    const Frame& frame, const ScenarioType& scenario_type) {
  auto* emergency_stop = injector_->planning_context()
                             ->mutable_planning_status()
                             ->mutable_emergency_stop();
  if (scenario_type != ScenarioType::EMERGENCY_STOP) {
    emergency_stop->Clear();
  }
}

// update: stop_sign status in PlanningContext
void ScenarioManager::UpdatePlanningContextStopSignScenario(
    const Frame& frame, const ScenarioType& scenario_type) {
  if (!IsStopSignScenario(scenario_type)) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // set to first_encountered stop_sign
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::STOP_SIGN);
  if (map_itr != first_encountered_overlap_map_.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->set_current_stop_sign_overlap_id(map_itr->second.object_id);
    ADEBUG << "Update PlanningContext with first_encountered stop sign["
           << map_itr->second.object_id << "] start_s["
           << map_itr->second.start_s << "]";
  }
}

// update: traffic_light(s) status in PlanningContext
void ScenarioManager::UpdatePlanningContextTrafficLightScenario(
    const Frame& frame, const ScenarioType& scenario_type) {
  if (!IsTrafficLightScenario(scenario_type)) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // get first_encountered traffic_light
  std::string current_traffic_light_overlap_id;
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::SIGNAL);
  if (map_itr != first_encountered_overlap_map_.end()) {
    current_traffic_light_overlap_id = map_itr->second.object_id;
  }

  if (current_traffic_light_overlap_id.empty()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return;
  }

  // find all the traffic light at/within the same location/group
  const auto& reference_line_info = frame.reference_line_info().front();
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_itr = std::find_if(
      traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
      [&current_traffic_light_overlap_id](const hdmap::PathOverlap& overlap) {
        return overlap.object_id == current_traffic_light_overlap_id;
      });
  if (traffic_light_overlap_itr == traffic_light_overlaps.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_traffic_light()
        ->Clear();
    return;
  }

  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  const double current_traffic_light_overlap_start_s =
      traffic_light_overlap_itr->start_s;
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    const double dist =
        traffic_light_overlap.start_s - current_traffic_light_overlap_start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_traffic_light()
          ->add_current_traffic_light_overlap_id(
              traffic_light_overlap.object_id);
      ADEBUG << "Update PlanningContext with first_encountered traffic_light["
             << traffic_light_overlap.object_id << "] start_s["
             << traffic_light_overlap.start_s << "]";
    }
  }
}

// update: yield_sign status in PlanningContext
void ScenarioManager::UpdatePlanningContextYieldSignScenario(
    const Frame& frame, const ScenarioType& scenario_type) {
  if (!IsYieldSignScenario(scenario_type)) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_yield_sign()
        ->Clear();
    return;
  }

  if (scenario_type == current_scenario_->scenario_type()) {
    return;
  }

  // get first_encountered yield_sign
  std::string current_yield_sign_overlap_id;
  const auto map_itr =
      first_encountered_overlap_map_.find(ReferenceLineInfo::YIELD_SIGN);
  if (map_itr != first_encountered_overlap_map_.end()) {
    current_yield_sign_overlap_id = map_itr->second.object_id;
  }

  if (current_yield_sign_overlap_id.empty()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_yield_sign()
        ->Clear();
    return;
  }

  // find all the yield_sign at/within the same location/group
  const auto& reference_line_info = frame.reference_line_info().front();
  const std::vector<PathOverlap>& yield_sign_overlaps =
      reference_line_info.reference_line().map_path().yield_sign_overlaps();
  auto yield_sign_overlap_itr = std::find_if(
      yield_sign_overlaps.begin(), yield_sign_overlaps.end(),
      [&current_yield_sign_overlap_id](const hdmap::PathOverlap& overlap) {
        return overlap.object_id == current_yield_sign_overlap_id;
      });
  if (yield_sign_overlap_itr == yield_sign_overlaps.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_yield_sign()
        ->Clear();
    return;
  }

  static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
  const double current_yield_sign_overlap_start_s =
      yield_sign_overlap_itr->start_s;
  for (const auto& yield_sign_overlap : yield_sign_overlaps) {
    const double dist =
        yield_sign_overlap.start_s - current_yield_sign_overlap_start_s;
    if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
      injector_->planning_context()
          ->mutable_planning_status()
          ->mutable_yield_sign()
          ->add_current_yield_sign_overlap_id(yield_sign_overlap.object_id);
      ADEBUG << "Update PlanningContext with first_encountered yield_sign["
             << yield_sign_overlap.object_id << "] start_s["
             << yield_sign_overlap.start_s << "]";
    }
  }
}

// update: pull_over status in PlanningContext
void ScenarioManager::UpdatePlanningContextPullOverScenario(
    const Frame& frame, const ScenarioType& scenario_type) {
  auto* pull_over = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_pull_over();
  if (scenario_type == ScenarioType::PULL_OVER) {
    pull_over->set_pull_over_type(PullOverStatus::PULL_OVER);
    pull_over->set_plan_pull_over_path(true);
    return;
  } else if (scenario_type == ScenarioType::EMERGENCY_PULL_OVER) {
    pull_over->set_pull_over_type(PullOverStatus::EMERGENCY_PULL_OVER);
    return;
  }

  pull_over->set_plan_pull_over_path(false);

  // check pull_over_status left behind
  // keep it if close to destination, to keep stop fence
  const auto& pull_over_status =
      injector_->planning_context()->planning_status().pull_over();
  if (pull_over_status.has_position() && pull_over_status.position().has_x() &&
      pull_over_status.position().has_y()) {
    const auto& routing = frame.local_view().routing;
    if (routing->routing_request().waypoint_size() >= 2) {
      // keep pull-over stop fence if destination not changed
      const auto& reference_line_info = frame.reference_line_info().front();
      const auto& reference_line = reference_line_info.reference_line();

      common::SLPoint dest_sl;
      const auto& routing_end =
          *(routing->routing_request().waypoint().rbegin());
      reference_line.XYToSL(routing_end.pose(), &dest_sl);

      common::SLPoint pull_over_sl;
      reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);

      static constexpr double kDestMaxDelta = 30.0;  // meter //
      if (std::fabs(dest_sl.s() - pull_over_sl.s()) > kDestMaxDelta) { //若当前场景不是pullover场景，那么如果距离终点稍远的话，会清掉pullover的状态
        injector_->planning_context()
            ->mutable_planning_status()
            ->clear_pull_over();
      }
    }
  }
}

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
