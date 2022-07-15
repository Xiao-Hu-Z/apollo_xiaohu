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
#include "modules/perception/lidar/lib/object_builder/object_builder.h"

#include <algorithm>
#include <limits>

#include "modules/perception/common/geometry/common.h"
#include "modules/perception/common/geometry/convex_hull_2d.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace lidar {

static const float kEpsilon = 1e-6f;
static const float kEpsilonForSize = 1e-2f;
static const float kEpsilonForLine = 1e-3f;
using apollo::perception::base::PointD;
using apollo::perception::base::PointF;
using ObjectPtr = std::shared_ptr<apollo::perception::base::Object>;
using PointFCloud = apollo::perception::base::PointCloud<PointF>;
using PolygonDType = apollo::perception::base::PointCloud<PointD>;

bool ObjectBuilder::Init(const ObjectBuilderInitOptions& options) {
  return true;
}

bool ObjectBuilder::Build(const ObjectBuilderOptions& options,LidarFrame* frame) {
  if (frame == nullptr) {
    return false;
  }
  std::vector<ObjectPtr>* objects = &(frame->segmented_objects);
  for (size_t i = 0; i < objects->size(); ++i) {
    if (objects->at(i)) {
      objects->at(i)->id = static_cast<int>(i);
      // // 计算点云拟合的凸多边形
      ComputePolygon2D(objects->at(i));
      // 计算3d bounding-box的点云的尺寸和中心
      ComputePolygonSizeCenter(objects->at(i));
      // 计算object其它的信息，包括：anchor_point,latest_tracked_time
      ComputeOtherObjectInformation(objects->at(i));
    }
  }
  return true;
}

void ObjectBuilder::ComputePolygon2D(ObjectPtr object) {
  Eigen::Vector3f min_pt;
  Eigen::Vector3f max_pt;
  PointFCloud& cloud = object->lidar_supplement.cloud;
  // 获取object点云的xyz最大和最小值
  GetMinMax3D(cloud, &min_pt, &max_pt);
  // PointPillarsDetection::GetObjects计算cloud.size()为8，代表8个顶点，走不到这部
  if (cloud.size() < 4u) {
    SetDefaultValue(min_pt, max_pt, object);
    return;
  }
  // 判断是否为线程扰动，不是是线性扰动，需要第一个点x加上kEpsilonForLine，第二个点y加上kEpsilonForLine
  LinePerturbation(&cloud);
  common::ConvexHull2D<PointFCloud, PolygonDType> hull;
  // 计算点云拟合的凸多边形顶点坐标，存储在polygon字段中
  hull.GetConvexHull(cloud, &(object->polygon));
}

void ObjectBuilder::ComputeOtherObjectInformation(ObjectPtr object) {
  object->anchor_point = object->center;
  double timestamp = 0.0;
  size_t num_point = object->lidar_supplement.cloud.size();
  for (size_t i = 0; i < num_point; ++i) {
    timestamp += object->lidar_supplement.cloud.points_timestamp(i);
  }
  if (num_point > 0) {
    timestamp /= static_cast<double>(num_point);
  }
  // 最近测量的时间戳，latest_tracked_time是必要的
  object->latest_tracked_time = timestamp;
}

void ObjectBuilder::ComputePolygonSizeCenter(ObjectPtr object) {
  if (object->lidar_supplement.cloud.size() < 4u) {
    return;
  }
  Eigen::Vector3f dir = object->direction;
  // 计算3d bounding-box的点云的尺寸和中心
  common::CalculateBBoxSizeCenter2DXY(object->lidar_supplement.cloud, dir,&(object->size), &(object->center));
  // PointPillarsDetection::GetObject函数中is_background设为false
  if (object->lidar_supplement.is_background) {
    float length = object->size(0);
    float width = object->size(1);
    Eigen::Matrix<float, 3, 1> ortho_dir(-object->direction(1),object->direction(0), 0.0);
    // 保证object的size中第一个值是最大的
    if (length < width) {
      object->direction = ortho_dir;
      object->size(0) = width;
      object->size(1) = length;
    }
  }
  // 对于小于kEpsilonForSize的size，将其设为大小kEpsilonForSize
  for (size_t i = 0; i < 3; ++i) {
    if (object->size(i) < kEpsilonForSize) { // kEpsilonForSize = 1e-2f
      object->size(i) = kEpsilonForSize;
    }
  }
  object->theta = static_cast<float>(atan2(object->direction(1), object->direction(0)));
}

void ObjectBuilder::SetDefaultValue(const Eigen::Vector3f& min_pt_in,
                                    const Eigen::Vector3f& max_pt_in,
                                    ObjectPtr object) {
  Eigen::Vector3f min_pt = min_pt_in;
  Eigen::Vector3f max_pt = max_pt_in;
  // handle degeneration case
  for (int i = 0; i < 3; i++) {
    if (max_pt[i] - min_pt[i] < kEpsilonForSize) {  // kEpsilonForSize = 1e-2f
      max_pt[i] = max_pt[i] + kEpsilonForSize / 2;
      min_pt[i] = min_pt[i] - kEpsilonForSize / 2;
    }
  }
  Eigen::Vector3f center((min_pt[0] + max_pt[0]) / 2,
                         (min_pt[1] + max_pt[1]) / 2,
                         (min_pt[2] + max_pt[2]) / 2);
  object->center = Eigen::Vector3d(static_cast<double>(center(0)),
                                   static_cast<double>(center(1)),
                                   static_cast<double>(center(2)));
  float length = max_pt[0] - min_pt[0];
  float width = max_pt[1] - min_pt[1];
  float height = max_pt[2] - min_pt[2];
  // 保证object的length大于width,按按最长边计算direction
  if (length < width) {
    object->size = Eigen::Vector3f(width, length, height);
    object->direction = Eigen::Vector3f(0.0, 1.0, 0.0);
  } else {
    object->size = Eigen::Vector3f(length, width, height);
    object->direction = Eigen::Vector3f(1.0, 0.0, 0.0);
  }
  // polygon
  // objiect  lidar_supplement.cloud.size()为8，这里走不到？？？
  if (object->lidar_supplement.cloud.size() < 4) {
    object->polygon.resize(4);
    object->polygon[0].x = static_cast<double>(min_pt[0]);
    object->polygon[0].y = static_cast<double>(min_pt[1]);
    object->polygon[0].z = static_cast<double>(min_pt[2]);

    object->polygon[1].x = static_cast<double>(max_pt[0]);
    object->polygon[1].y = static_cast<double>(min_pt[1]);
    object->polygon[1].z = static_cast<double>(min_pt[2]);

    object->polygon[2].x = static_cast<double>(max_pt[0]);
    object->polygon[2].y = static_cast<double>(max_pt[1]);
    object->polygon[2].z = static_cast<double>(min_pt[2]);

    object->polygon[3].x = static_cast<double>(min_pt[0]);
    object->polygon[3].y = static_cast<double>(max_pt[1]);
    object->polygon[3].z = static_cast<double>(min_pt[2]);
  }
}


bool ObjectBuilder::LinePerturbation(PointFCloud* cloud) {
  if (cloud->size() >= 3) {
    int start_point = 0;
    int end_point = 1;
    // 第一个与第二个点x,y差
    float diff_x = cloud->at(start_point).x - cloud->at(end_point).x;
    float diff_y = cloud->at(start_point).y - cloud->at(end_point).y;
    size_t idx = 0;
    for (idx = 2; idx < cloud->size(); ++idx) {
      // 计算每个点与第一个点坐标 x,y 差
      float tdiff_x = cloud->at(idx).x - cloud->at(start_point).x;
      float tdiff_y = cloud->at(idx).y - cloud->at(start_point).y;
      // 判断是否为线程扰动
      if (fabs(diff_x * tdiff_y - tdiff_x * diff_y) > kEpsilonForLine) {
        return false;
      }
    }
    // 是线性扰动，第一个点x加上kEpsilonForLine，第二个点y加上kEpsilonForLine
    cloud->at(0).x += kEpsilonForLine;  // kEpsilonForLine = 1e-3f
    cloud->at(1).y += kEpsilonForLine;
    return true;
  }
  return true;
}

void ObjectBuilder::GetMinMax3D(const PointFCloud& cloud,
                                Eigen::Vector3f* min_pt,
                                Eigen::Vector3f* max_pt) {
  (*min_pt)[0] = (*min_pt)[1] = (*min_pt)[2] =
      std::numeric_limits<float>::max();
  (*max_pt)[0] = (*max_pt)[1] = (*max_pt)[2] =
      -std::numeric_limits<float>::max();
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (std::isnan(cloud[i].x) || std::isnan(cloud[i].y) ||
        std::isnan(cloud[i].z)) {
      continue;
    }
    (*min_pt)[0] = std::min((*min_pt)[0], cloud[i].x);
    (*max_pt)[0] = std::max((*max_pt)[0], cloud[i].x);
    (*min_pt)[1] = std::min((*min_pt)[1], cloud[i].y);
    (*max_pt)[1] = std::max((*max_pt)[1], cloud[i].y);
    (*min_pt)[2] = std::min((*min_pt)[2], cloud[i].z);
    (*max_pt)[2] = std::max((*max_pt)[2], cloud[i].z);
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
