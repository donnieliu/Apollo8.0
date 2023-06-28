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

#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {
//PathDecider是lanefollow场景下，所调用的第7个task，属于task中的decider类别，它的主要作用是：
//在上一个任务中获得了最优的路径，PathDecider的功能是根据静态障碍物做出自车的决策，对于前方的静态障碍物时忽略，stop还是nudge
class PathDecider : public Task {
 public:
  PathDecider(const TaskConfig &config,
              const std::shared_ptr<DependencyInjector> &injector);

  apollo::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  apollo::common::Status Process(const ReferenceLineInfo *reference_line_info,
                                 const PathData &path_data,
                                 PathDecision *const path_decision);

  bool MakeObjectDecision(const PathData &path_data,
                          const std::string &blocking_obstacle_id,
                          PathDecision *const path_decision);

  bool MakeStaticObstacleDecision(const PathData &path_data,
                                  const std::string &blocking_obstacle_id,
                                  PathDecision *const path_decision);

  ObjectStop GenerateObjectStopDecision(const Obstacle &obstacle) const;
};

}  // namespace planning
}  // namespace apollo
