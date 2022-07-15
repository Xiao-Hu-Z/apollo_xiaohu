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
#include "modules/perception/lidar/lib/map_manager/map_manager.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/proto/map_manager_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using cyber::common::GetAbsolutePath;

bool MapManager::Init(const MapManagerInitOptions& options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  ACHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  ACHECK(model_config->get_value("root_path", &root_path));
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, "map_manager.conf");
  MapManagerConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
  update_pose_ = config.update_pose();
  roi_search_distance_ = config.roi_search_distance();
  hdmap_input_ = map::HDMapInput::Instance();
  if (!hdmap_input_->Init()) {
    AINFO << "Failed to init hdmap input.";
    return false;
  }
  return true;
}

bool MapManager::Update(const MapManagerOptions& options, LidarFrame* frame) {
  if (!frame) {// 判断点云是否为空
    AINFO << "Frame is nullptr.";
    return false;
  }
  if (!(frame->hdmap_struct)) {
    frame->hdmap_struct.reset(new base::HdmapStruct); // 重置地图
  }
  if (!hdmap_input_)
  { // 看看输入的地图是否为空初始化的时候给它赋的值
    AINFO << "Hdmap input is nullptr";
    return false;
  }
  if (update_pose_) // 是否需要更新位置
  {                 // 获取自身定位
    if (!QueryPose(&(frame->lidar2world_pose))) {
      AINFO << "Failed to query updated pose.";
    }
  }

  base::PointD point; // 设置一个point接收定位信息
  point.x = frame->lidar2world_pose.translation()(0);
  point.y = frame->lidar2world_pose.translation()(1);
  point.z = frame->lidar2world_pose.translation()(2);

  /*
  获取高精地图中，离我们所在定位位置距离为roi_search_distance_的所有道路信息
  的road_polygons、road_boundary、hole_polygons、junction_polygons存到frame里
  如果没有则hdmap_input_->GetRoiHDMapStruct（）返回false，所有的道路信息置空
  */
  if (!hdmap_input_->GetRoiHDMapStruct(point, roi_search_distance_,frame->hdmap_struct)) {
    // 路面的polygons多边形信息
    frame->hdmap_struct->road_polygons.clear();
    // 道路边界线
    frame->hdmap_struct->road_boundary.clear();
    // hole_polygonsm 没太看懂，没怎么用到这个信息
    frame->hdmap_struct->hole_polygons.clear();
    // 交叉路口多边形信息
    frame->hdmap_struct->junction_polygons.clear();
    AINFO << "Failed to get roi from hdmap.";
  }
  return true;
}
bool MapManager::QueryPose(Eigen::Affine3d* sensor2world_pose) const {
  // TODO(...): map-based alignment to refine pose
  // 需要我们自己写的，获取当前的位置,设计到定位算法，NDT点云定位
  return false;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
