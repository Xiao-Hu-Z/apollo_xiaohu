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
#include "modules/perception/lidar/lib/object_filter_bank/object_filter_bank.h"

#include "cyber/common/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/object_filter_bank/proto/filter_bank_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::cyber::common::GetAbsolutePath;

bool ObjectFilterBank::Init(const ObjectFilterInitOptions& options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  // 第一次调用GetModelConfig已经初始化，直接从字典model_config_map_查找"ObjectFilterBank"文件的信息，以proto存储的
  // apollo/modules/perception/production/conf/perception/lidar/modules/object_filter_bank.config
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  // ./data/perception/lidar/models/object_filter_bank
  ACHECK(model_config->get_value("root_path", &root_path));
  // apollo/modules/perception/production//data/perception/lidar/models/object_filter_bank
  config_file = GetAbsolutePath(work_root, root_path);
  // apollo/modules/perception/production/data/perception/lidar/models/object_filter_bank/velodyne128
  config_file = GetAbsolutePath(config_file, options.sensor_name);
  // apollo/modules/perception/production/data/perception/lidar/models/object_filter_bank/velodyne128/filter_bank.conf
  config_file = GetAbsolutePath(config_file, "filter_bank.conf");
  // proto定义message文件路径：pollo/modules/perception/lidar/lib/object_filter_bank/proto/filter_bank_config.proto
  FilterBankConfig config;
  // 用定义的meaage读取filter_bank.conf文件信息  
  ACHECK(apollo::cyber::common::GetProtoFromFile(config_file, &config));
  filter_bank_.clear();
  for (int i = 0; i < config.filter_name_size(); ++i) {
    const auto& name = config.filter_name(i); // ROIBoundaryFilter
    // 通过工厂方法模式获取 ROIBoundaryFilter 类的实例指针，也是多态运用
    BaseObjectFilter* filter = BaseObjectFilterRegisterer::GetInstanceByName(name);
    if (!filter) {
      AINFO << "Failed to find object filter: " << name << ", skipped";
      continue;
    }
    // 调用ROIBoundaryFilter::Init函数
    if (!filter->Init()) {
      AINFO << "Failed to init object filter: " << name << ", skipped";
      continue;
    }
    filter_bank_.push_back(filter);
    AINFO << "Filter bank add filter: " << name;
  }
  return true;
}

bool ObjectFilterBank::Filter(const ObjectFilterOptions& options,
                              LidarFrame* frame) {
  size_t object_number = frame->segmented_objects.size(); // 包围框bounding_box的数量
  for (auto& filter : filter_bank_) {
    // 调用ROIBoundaryFilter::Filter方法
    if (!filter->Filter(options, frame)) {
      AINFO << "Failed to filter objects in: " << filter->Name(); // ROIBoundaryFilter
    }
  }
  AINFO << "Object filter bank, filtered objects size: from " << object_number
        << " to " << frame->segmented_objects.size();
  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
