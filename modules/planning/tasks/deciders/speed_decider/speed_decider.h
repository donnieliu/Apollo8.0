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

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "modules/planning/common/frame.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {
//SpeedDecider是lanefollow场景下，Apollo Planning算法所调用的第13个task，属于task的decider类别，它的主要作用是：
//1，对每个目标进行遍历，分别对每个目标进行决策；
//2，获得mutable_obstacle->path_st_boundary()
//3，根据障碍物st_boundary的时间与位置的分布，判断是否要忽略；
//4，对于虚拟目标Virtual obstacle，如果不在referenceline的车道上，则跳过；
//5，如果是行人则决策结果置位stop；
//6，SpeedDecider::GetSTLocation()获取障碍物在st图上与自车路径的位置关系；
//7，根据不同的STLocation，来对障碍物进行决策；
//8，如果没有纵向决策结果，则位置 ignore_decison;
class SpeedDecider : public Task {
 public:
  SpeedDecider(const TaskConfig& config,
               const std::shared_ptr<DependencyInjector>& injector);

  common::Status Execute(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

 private:
  enum STLocation {
    ABOVE = 1,
    BELOW = 2,
    CROSS = 3,
  };

  STLocation GetSTLocation(const PathDecision* const path_decision,
                           const SpeedData& speed_profile,
                           const STBoundary& st_boundary) const;

  bool CheckKeepClearCrossable(const PathDecision* const path_decision,
                               const SpeedData& speed_profile,
                               const STBoundary& keep_clear_st_boundary) const;

  bool CheckKeepClearBlocked(const PathDecision* const path_decision,
                             const Obstacle& keep_clear_obstacle) const;

  /**
   * @brief check if the ADC should follow an obstacle by examing the
   *StBoundary of the obstacle.
   * @param boundary The boundary of the obstacle.
   * @return true if the ADC believe it should follow the obstacle, and
   *         false otherwise.
   **/
  bool CheckIsFollow(const Obstacle& obstacle,
                     const STBoundary& boundary) const;

  bool CheckStopForPedestrian(const Obstacle& obstacle) const;

  bool CreateStopDecision(const Obstacle& obstacle,
                          ObjectDecisionType* const stop_decision,
                          double stop_distance) const;

  /**
   * @brief create follow decision based on the boundary
   **/
  bool CreateFollowDecision(const Obstacle& obstacle,
                            ObjectDecisionType* const follow_decision) const;

  /**
   * @brief create yield decision based on the boundary
   **/
  bool CreateYieldDecision(const Obstacle& obstacle,
                           ObjectDecisionType* const yield_decision) const;

  /**
   * @brief create overtake decision based on the boundary
   **/
  bool CreateOvertakeDecision(
      const Obstacle& obstacle,
      ObjectDecisionType* const overtake_decision) const;

  common::Status MakeObjectDecision(const SpeedData& speed_profile,
                                    PathDecision* const path_decision) const;

  void AppendIgnoreDecision(Obstacle* obstacle) const;

  /**
   * @brief "too close" is determined by whether ego vehicle will hit the front
   * obstacle if the obstacle drive at current speed and ego vehicle use some
   * reasonable deceleration
   **/
  bool IsFollowTooClose(const Obstacle& obstacle) const;

 private:
  SLBoundary adc_sl_boundary_;
  common::TrajectoryPoint init_point_;
  const ReferenceLine* reference_line_ = nullptr;
};

}  // namespace planning
}  // namespace apollo
