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
#include "modules/perception/lidar/lib/object_filter_bank/roi_boundary_filter/roi_boundary_filter.h"

#include <limits>

#include "cyber/common/file.h"
#include "cyber/common/log.h"

#include "modules/perception/common/geometry/common.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/proto/roi_boundary_filter_config.pb.h"

using apollo::common::EigenVector;

namespace
{
  constexpr double kDoubleMax = std::numeric_limits<double>::max();
} // namespace

namespace apollo
{
  namespace perception
  {
    namespace lidar
    {

      using cyber::common::GetAbsolutePath;

      bool ROIBoundaryFilter::Init(const ObjectFilterInitOptions &options)
      {
        auto config_manager = lib::ConfigManager::Instance();
        const lib::ModelConfig *model_config = nullptr;
        // 第一次调用GetModelConfig已经初始化，直接从字典model_config_map_查找"ROIBoundaryFilter"文件的信息，以proto存储的
        // apollo/modules/perception/production/conf/perception/lidar/modules/roi_boundary_filter.config
        ACHECK(config_manager->GetModelConfig(Name(), &model_config));
        const std::string work_root = config_manager->work_root();
        std::string config_file;
        zhius
            std::string root_path;
        // data/perception/lidar/models/object_filter_bank
        ACHECK(model_config->get_value("root_path", &root_path));
        // apollo/modules/perception/production/data/perception/lidar/models/object_filter_bank
        config_file = GetAbsolutePath(work_root, root_path);
        // // apollo/modules/perception/production/data/perception/lidar/models/object_filter_bank/roi_boundary_filter.conf
        config_file = GetAbsolutePath(config_file, "roi_boundary_filter.conf");
        // proto定义message文件路径：apollo/modules/perception/proto/roi_boundary_filter_config.proto
        ROIBoundaryFilterConfig config;
        // 用定义的meaage读取roi_boundary_filter.conf文件信息
        ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
        distance_to_boundary_threshold_ = config.distance_to_boundary_threshold(); // 1.0
        confidence_threshold_ = config.confidence_threshold();                     // 0.5  用于在后期处理过程中滤出候选bounding_box的检测置信度得分阈值
        cross_roi_threshold_ = config.cross_roi_threshold();                       // 0.6
        inside_threshold_ = config.inside_threshold();                             // 1.0
        return true;
      }

      bool ROIBoundaryFilter::Filter(const ObjectFilterOptions &options, LidarFrame *frame)
      {
        if (!frame)
        {
          AINFO << "Lidar frame is nullptr.";
          return false;
        }
        if (!frame->hdmap_struct)
        {
          AINFO << "HDMap struct is nullptr.";
          return true;
        }
        if (frame->hdmap_struct->road_boundary.size() + frame->hdmap_struct->road_polygons.size() + frame->hdmap_struct->junction_polygons.size() == 0)
        {
          AINFO << "Donot find roi polygons, skip boundary filter.";
          for (auto &object : frame->segmented_objects)
          {
            object->lidar_supplement.is_in_roi = true;
          }
          return true;
        }
        auto &objects = frame->segmented_objects;
        for (auto &obj : objects)
        {
          // PointPillarsDetection::GetObject函数中is_background设为false
          obj->lidar_supplement.is_in_roi = obj->lidar_supplement.is_background;
        }
        // objects_cross_roi_ 赋值，bool,object是否在交叉路口区域
        FillObjectRoiFlag(options, frame);
        objects_valid_flag_.assign(frame->segmented_objects.size(), true);
        BuildWorldPolygons(options, *frame);
        // distance_to_boundary_threshold_ = 1.0
        // 过滤道路边界线外的目标
        if (distance_to_boundary_threshold_ >= 0.f)
        {
          FilterObjectsOutsideBoundary(options, frame, &objects_valid_flag_);
        }
        if (inside_threshold_ >= 0.f)
        {
          FilterObjectsInsideBoundary(options, frame, &objects_valid_flag_);
        }
        FilterObjectsByConfidence(options, frame, &objects_valid_flag_);
        size_t count = 0;
        for (size_t i = 0; i < objects.size(); ++i)
        {
          if (objects_valid_flag_[i])
          {
            if (count != i)
            {
              objects[count] = objects[i];
            }
            ++count;
          }
        }
        objects.resize(count);
        AINFO << "Roi boundary filter, " << objects_valid_flag_.size() << " to "
              << count;
        return true;
      }

      void ROIBoundaryFilter::BuildWorldPolygons(const ObjectFilterOptions &options,
                                                 const LidarFrame &frame)
      {
        const Eigen::Affine3d &pose = frame.lidar2world_pose;
        const std::vector<base::ObjectPtr> &objects = frame.segmented_objects;
        polygons_in_world_.clear();
        polygons_in_world_.resize(objects.size());
        Eigen::Vector3d local_point;
        Eigen::Vector3d world_point;
        for (size_t i = 0; i < objects.size(); ++i)
        {
          const base::ObjectPtr &obj = objects[i];
          if (!obj->lidar_supplement.is_background)
          {
            polygons_in_world_[i].resize(obj->polygon.size());
            for (size_t j = 0; j < obj->polygon.size(); ++j)
            {
              local_point(0) = obj->polygon[j].x;
              local_point(1) = obj->polygon[j].y;
              local_point(2) = obj->polygon[j].z;
              world_point = pose * local_point;
              polygons_in_world_[i][j].x = world_point(0);
              polygons_in_world_[i][j].y = world_point(1);
              polygons_in_world_[i][j].z = world_point(2);
            }
          }
        }
      }

      void ROIBoundaryFilter::FillObjectRoiFlag(const ObjectFilterOptions &options,
                                                LidarFrame *frame)
      {
        auto &objects = frame->segmented_objects;
        objects_cross_roi_.assign(objects.size(), false);
        for (size_t i = 0; i < objects.size(); ++i)
        {
          auto &obj = objects[i];
          if (obj->lidar_supplement.is_in_roi)
          {
            continue;
          }
          // num_points_in_roi 为8
          obj->lidar_supplement.is_in_roi = obj->lidar_supplement.num_points_in_roi > 0;                // true
          bool cross = (obj->lidar_supplement.num_points_in_roi != obj->lidar_supplement.cloud.size()); // false
          float ratio = static_cast<float>(obj->lidar_supplement.num_points_in_roi) / static_cast<float>(obj->lidar_supplement.cloud.size());
          // cross_roi_threshold_ 0.6
          if (ratio < cross_roi_threshold_ || (cross && obj->confidence <= .11f))
          { // a hacked minimum support value
            objects_cross_roi_[i] = true;
          }
        }
      }

      void ROIBoundaryFilter::FilterObjectsOutsideBoundary(const ObjectFilterOptions &options, LidarFrame *frame, std::vector<bool> *objects_valid_flag)
      {
        const EigenVector<base::RoadBoundary> &road_boundary = frame->hdmap_struct->road_boundary;
        auto &objects = frame->segmented_objects;
        double dist_to_boundary = 0.0;
        Eigen::Vector3d direction;
        double min_dist_to_boundary = kDoubleMax;
        Eigen::Vector3d world_point;
        for (size_t i = 0; i < objects.size(); ++i)
        {
          auto &obj = objects[i];
          // only compute distance for those outside roi
          if (!obj->lidar_supplement.is_in_roi)
          {
            (*objects_valid_flag)[i] = false;
            for (auto &point : polygons_in_world_[i].points())
            {
              dist_to_boundary = 0.0;
              min_dist_to_boundary = kDoubleMax;
              world_point << point.x, point.y, point.z;
              for (const auto &boundary : road_boundary)
              {
                perception::common::CalculateDistAndDirToBoundary(
                    world_point, boundary.left_boundary, boundary.right_boundary,
                    &dist_to_boundary, &direction);
                if (min_dist_to_boundary > dist_to_boundary)
                {
                  min_dist_to_boundary = dist_to_boundary;
                }
              }
              if (min_dist_to_boundary <= distance_to_boundary_threshold_)
              {
                (*objects_valid_flag)[i] = true;
                break;
              }
            }
            if (!(*objects_valid_flag)[i])
            {
              ADEBUG << "Roi boundary filter: min_dist_to_boundary exceed "
                     << distance_to_boundary_threshold_ << ", id " << obj->id
                     << ", center " << obj->center.head<2>().transpose()
                     << ", distance " << min_dist_to_boundary;
            }
          }
        }
      }

      void ROIBoundaryFilter::FilterObjectsInsideBoundary(
          const ObjectFilterOptions &options, LidarFrame *frame,
          std::vector<bool> *objects_valid_flag)
      {
        const EigenVector<base::RoadBoundary> &road_boundary =
            frame->hdmap_struct->road_boundary;
        auto &objects = frame->segmented_objects;
        double dist_to_boundary = 0.0;
        Eigen::Vector3d direction;
        double min_dist_to_boundary = kDoubleMax;
        Eigen::Vector3d world_point;
        for (size_t i = 0; i < objects.size(); ++i)
        {
          auto &obj = objects[i];
          // only compute distance for those outside roi
          if (obj->lidar_supplement.is_in_roi && !objects_cross_roi_[i] &&
              obj->confidence <= .11f)
          {
            (*objects_valid_flag)[i] = false;
            for (auto &point : polygons_in_world_[i].points())
            {
              dist_to_boundary = 0.0;
              min_dist_to_boundary = kDoubleMax;
              world_point << point.x, point.y, point.z;
              for (auto &boundary : road_boundary)
              {
                perception::common::CalculateDistAndDirToBoundary(
                    world_point, boundary.left_boundary, boundary.right_boundary,
                    &dist_to_boundary, &direction);
                if (min_dist_to_boundary > dist_to_boundary)
                {
                  min_dist_to_boundary = dist_to_boundary;
                }
              }
              if (min_dist_to_boundary > inside_threshold_)
              {
                (*objects_valid_flag)[i] = true;
                break;
              }
            }
            if (!(*objects_valid_flag)[i])
            {
              ADEBUG << "Roi boundary filter: inside_distance within "
                     << inside_threshold_ << ", id " << obj->id << ", center "
                     << obj->center.head<2>().transpose() << ", distance "
                     << min_dist_to_boundary;
            }
          }
        }
      }

      void ROIBoundaryFilter::FilterObjectsByConfidence(
          const ObjectFilterOptions &options, LidarFrame *frame,
          std::vector<bool> *objects_valid_flag)
      {
        auto &objects = frame->segmented_objects;
        for (size_t i = 0; i < objects.size(); ++i)
        {
          if (objects_cross_roi_[i] || !objects[i]->lidar_supplement.is_in_roi)
          {
            if (objects[i]->confidence < confidence_threshold_)
            {
              ADEBUG << "Roi boundary filter: confidence " << objects[i]->confidence
                     << " below " << confidence_threshold_ << ", id "
                     << objects[i]->id << ", center "
                     << objects[i]->center.head<2>().transpose() << " cross roi "
                     << objects_cross_roi_[i] << " in roi "
                     << objects[i]->lidar_supplement.is_in_roi << " #points_in_roi "
                     << objects[i]->lidar_supplement.num_points_in_roi << "/"
                     << objects[i]->lidar_supplement.cloud.size();
              (*objects_valid_flag)[i] = false;
            }
          }
        }
      }

      PERCEPTION_REGISTER_OBJECTFILTER(ROIBoundaryFilter);

    } // namespace lidar
  }   // namespace perception
} // namespace apollo
