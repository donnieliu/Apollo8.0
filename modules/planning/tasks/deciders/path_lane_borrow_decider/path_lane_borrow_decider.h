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

/**
 * @file
 **/

#pragma once

#include <memory>

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/deciders/decider.h"
//PathLaneBorrowDecider是第三个task，PathLaneBorrowDecider会判断已处于借道场景下判断是否退出避让；判断未处于借道场景下判断是否具备借道能力；
//PathLaneBorrowDecider只是判断是否满足借道条件，具体的轨迹是否借道，由后面的task决定；
namespace apollo {
namespace planning {

class PathLaneBorrowDecider : public Decider {
 public:
  PathLaneBorrowDecider(const TaskConfig& config,
                        const std::shared_ptr<DependencyInjector>& injector);

 private:
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info) override;

  bool IsNecessaryToBorrowLane(const Frame& frame,
                               const ReferenceLineInfo& reference_line_info);

  bool HasSingleReferenceLine(const Frame& frame);

  bool IsWithinSidePassingSpeedADC(const Frame& frame);

  bool IsLongTermBlockingObstacle();

  bool IsBlockingObstacleWithinDestination(
      const ReferenceLineInfo& reference_line_info);

  bool IsBlockingObstacleFarFromIntersection(
      const ReferenceLineInfo& reference_line_info);

  bool IsSidePassableObstacle(const ReferenceLineInfo& reference_line_info);

  void CheckLaneBorrow(const ReferenceLineInfo& reference_line_info,
                       bool* left_neighbor_lane_borrowable,
                       bool* right_neighbor_lane_borrowable);
};

}  // namespace planning
}  // namespace apollo
