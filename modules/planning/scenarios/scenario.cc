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

/**
 * @file
 **/

#include "modules/planning/scenarios/scenario.h"

#include "cyber/common/file.h"
#include "modules/planning/common/frame.h"

namespace apollo {
namespace planning {
namespace scenario {

Scenario::Scenario(const ScenarioConfig& config, const ScenarioContext* context,
                   const std::shared_ptr<DependencyInjector>& injector)
    : config_(config), scenario_context_(context), injector_(injector) {
  name_ = ScenarioType_Name(config.scenario_type());
}

bool Scenario::LoadConfig(const std::string& config_file,
                          ScenarioConfig* config) {
  return apollo::cyber::common::GetProtoFromFile(config_file, config);
}

void Scenario::Init() {
  ACHECK(!config_.stage_type().empty());

  // set scenario_type in PlanningContext
  auto* scenario = injector_->planning_context()
                       ->mutable_planning_status()
                       ->mutable_scenario();
  scenario->Clear();
  scenario->set_scenario_type(scenario_type());

  for (const auto& stage_config : config_.stage_config()) {
    stage_config_map_[stage_config.stage_type()] = &stage_config;
  }
  for (int i = 0; i < config_.stage_type_size(); ++i) {
    auto stage_type = config_.stage_type(i);
    ACHECK(common::util::ContainsKey(stage_config_map_, stage_type))
        << "stage type : " << StageType_Name(stage_type)
        << " has no config";
  }
  ADEBUG << "init stage "
         << StageType_Name(config_.stage_type(0));
  current_stage_ =     //初始化后分配该场景下默认的stage
      CreateStage(*stage_config_map_[config_.stage_type(0)], injector_);
}

Scenario::ScenarioStatus Scenario::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  if (current_stage_ == nullptr) {    //stage类型unknow
    AWARN << "Current stage is a null pointer.";
    return STATUS_UNKNOWN;
  }
  if (current_stage_->stage_type() == StageType::NO_STAGE) {  //stage全部执行完
    scenario_status_ = STATUS_DONE;
    return scenario_status_;
  }
  //stage::Process主要逻辑是根据配置文件，来依次执行每一个注册的task的Execute函数，从而把具体的规划任务分散到
  //每个task中stage并不属于某个特定的Scenario，task也不属于某个特定的stage，这些任务的组合通过配置文件进行配置，
  //这样保证了整个规划模块的解耦和扩展。
  auto ret = current_stage_->Process(planning_init_point, frame);
  switch (ret) {  //读取当前stage完成的状态，并进行处理
    case Stage::ERROR: {
      AERROR << "Stage '" << current_stage_->Name() << "' returns error";
      scenario_status_ = STATUS_UNKNOWN;
      break;
    }
    case Stage::RUNNING: {
      scenario_status_ = STATUS_PROCESSING;
      break;
    }
    case Stage::FINISHED: {
      auto next_stage = current_stage_->NextStage();
      if (next_stage != current_stage_->stage_type()) {
        AINFO << "switch stage from " << current_stage_->Name() << " to "
              << StageType_Name(next_stage);
        if (next_stage == StageType::NO_STAGE) {
          scenario_status_ = STATUS_DONE;
          return scenario_status_;
        }
        if (stage_config_map_.find(next_stage) == stage_config_map_.end()) {
          AERROR << "Failed to find config for stage: " << next_stage;
          scenario_status_ = STATUS_UNKNOWN;
          return scenario_status_;
        }
        current_stage_ = CreateStage(*stage_config_map_[next_stage], injector_);
        if (current_stage_ == nullptr) {
          AWARN << "Current stage is a null pointer.";
          return STATUS_UNKNOWN;
        }
      }
      if (current_stage_ != nullptr &&
          current_stage_->stage_type() != StageType::NO_STAGE) {
        scenario_status_ = STATUS_PROCESSING;
      } else {
        scenario_status_ = STATUS_DONE;
      }
      break;
    }
    default: {
      AWARN << "Unexpected Stage return value: " << ret;
      scenario_status_ = STATUS_UNKNOWN;
    }
  }
  return scenario_status_;
}

const std::string& Scenario::Name() const { return name_; }

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
