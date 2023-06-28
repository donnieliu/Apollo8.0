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

#pragma once

#include <memory>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/message/raw_message.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
#include "modules/planning/common/message_process.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/planning_base.h"
#include "modules/planning/proto/learning_data.pb.h"
#include "modules/common_msgs/planning_msgs/pad_msg.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/common_msgs/storytelling_msgs/story.pb.h"

namespace apollo {
namespace planning {
//要先把class定义为ComponentBase Class的subclass (Component和TimerComponent都是ComponentBase的sub class).
class PlanningComponent final
    : public cyber::Component<prediction::PredictionObstacles, canbus::Chassis,
                              localization::LocalizationEstimate> {
 public:
  PlanningComponent() = default;

  ~PlanningComponent() = default;

 public:
  bool Init() override;

  bool Proc(const std::shared_ptr<prediction::PredictionObstacles>&
                prediction_obstacles,
            const std::shared_ptr<canbus::Chassis>& chassis,
            const std::shared_ptr<localization::LocalizationEstimate>&
                localization_estimate) override;

 private:
  void CheckRerouting();
  bool CheckInput();

 private:
  std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>>
      traffic_light_reader_;
  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
  std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;
  std::shared_ptr<cyber::Reader<storytelling::Stories>> story_telling_reader_;

  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;
  std::shared_ptr<cyber::Writer<PlanningLearningData>>
      planning_learning_data_writer_;

  std::mutex mutex_;
  perception::TrafficLightDetection traffic_light_;
  routing::RoutingResponse routing_;
  planning::PadMessage pad_msg_;
  relative_map::MapMsg relative_map_;
  storytelling::Stories stories_;

  LocalView local_view_;

  std::unique_ptr<PlanningBase> planning_base_;
  std::shared_ptr<DependencyInjector> injector_;
//PlanningConfig 用于构造 Planning 模型，包括 topic / task 列表，以及规划器的类型；
//还定义了 ScenarioConfig 消息体，描述当前处于什么场景的什么阶段。
  PlanningConfig config_;
  MessageProcess message_process_;
};
// 在Cyber中注册模块,CYBER_REGISTER_COMPONENT，是把一个用来产生模块object的class factory，加到了cyber的一个单例对象中了.
CYBER_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning
}  // namespace apollo
