/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/lidar/app/lidar_obstacle_detection.h"

#include "cyber/common/file.h"
#include "modules/common/util/perf_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/app/proto/lidar_obstacle_detection_config.pb.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/scene_manager/scene_manager.h"

namespace apollo
{
  namespace perception
  {
    namespace lidar
    {

      bool LidarObstacleDetection::Init(const LidarObstacleDetectionInitOptions &options)
      {
        auto &sensor_name = options.sensor_name; // 传感器名，如"velodyne128"
        // ConfigManager类经宏定义 DECLARE_SINGLETON(ConfigManager) 修饰成为单例类
        auto config_manager = lib::ConfigManager::Instance();
        const lib::ModelConfig *model_config = nullptr;
        // 获取 LidarObstacleDetection 配置参数model_config,第一次调用GetModelConfig将各模块功能类和其配置参数存储在字典，从字典查找LidarObstacleDetection对应的参数信息
        // LidarObstacleDetection在文件中是存储在 apollo/modules/perception/production/conf/perception/lidar/modules/lidar_obstacle_pipeline.config
        ACHECK(config_manager->GetModelConfig(Name(), &model_config));

        const std::string work_root = config_manager->work_root();
        std::string config_file;
        std::string root_path;
        // root_path:./data/perception/lidar/models/lidar_obstacle_pipeline
        ACHECK(model_config->get_value("root_path", &root_path));
        // apollo/modules/perception/production/lidar/models/lidar_obstacle_pipeline
        config_file = cyber::common::GetAbsolutePath(work_root, root_path);
        // apollo/modules/perception/production/data/perception/lidar/models/lidar_obstacle_pipeline/velodyne128
        config_file = cyber::common::GetAbsolutePath(config_file, sensor_name);
        // apollo/modules/perception/production/data/perception/lidar/models/lidar_obstacle_pipeline/velodyne128/lidar_obstacle_detection.conf
        config_file = cyber::common::GetAbsolutePath(config_file, "lidar_obstacle_detection.conf");
        /*
        message LidarObstacleDetectionConfig {
        optional string preprocessor = 1 [default = "PointCloudPreprocessor"];
        optional string detector = 2 [default = "PointPillarsDetection"];
        optional bool use_map_manager = 3 [default = true];
        optional bool use_object_filter_bank = 4 [default = true];
        }
        */
        LidarObstacleDetectionConfig config;
        // 把lidar_obstacle_detection.conf写入proto LidarObstacleDetectionConfig信息中
        ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
        use_map_manager_ = config.use_map_manager();               // true
        use_object_filter_bank_ = config.use_object_filter_bank(); // true
        // PointPillarsDetection
        use_object_builder_ = ("PointPillarsDetection" != config.detector() ||
                               "MaskPillarsDetection" != config.detector());

        use_map_manager_ = use_map_manager_ && options.enable_hdmap_input; // true

        SceneManagerInitOptions scene_manager_init_options;
        ACHECK(SceneManager::Instance().Init(scene_manager_init_options));
        // 是否使用高精度地图
        if (use_map_manager_)
        {
          MapManagerInitOptions map_manager_init_options;
          // hdmap初始化
          if (!map_manager_.Init(map_manager_init_options))
          {
            AINFO << "Failed to init map manager.";
            use_map_manager_ = false;
          }
        }
        // 激光点云预处理：初始化基类对象，让其指针指向子类
        BasePointCloudPreprocessor *preprocessor = BasePointCloudPreprocessorRegisterer::GetInstanceByName(config.preprocessor());
        CHECK_NOTNULL(preprocessor);
        cloud_preprocessor_.reset(preprocessor);

        // 激光雷达的型号
        PointCloudPreprocessorInitOptions preprocessor_init_options;
        preprocessor_init_options.sensor_name = sensor_name;
        // 点云预处理初始化
        ACHECK(cloud_preprocessor_->Init(preprocessor_init_options)) << "lidar preprocessor init error";
        // 激光障碍物检测
        BaseLidarDetector *detector = BaseLidarDetectorRegisterer::GetInstanceByName(config.detector());
        BaseLidarDetectorRegisterer::GetInstanceByName(config.detector());
        detector_.reset(detector);
        LidarDetectorInitOptions detection_init_options;
        detection_init_options.sensor_name = sensor_name;
        // 激光雷达障碍物检测初始化
        ACHECK(detector_->Init(detection_init_options)) << "lidar detector init error";

        if (use_object_builder_)
        {
          // ObjectBuilder：构建障碍物目标包围框类信息
          ObjectBuilderInitOptions builder_init_options;
          ACHECK(builder_.Init(builder_init_options));
        }

        if (use_object_filter_bank_)
        {
          ObjectFilterInitOptions filter_bank_init_options;// 定义参数
          filter_bank_init_options.sensor_name = sensor_name; // 传感器名
          // ObjectFilterBank： 调用ObjectFilterBank：对目标进行ROIBoundaryFilter
          ACHECK(filter_bank_.Init(filter_bank_init_options));
        }

        return true;
      }

      LidarProcessResult LidarObstacleDetection::Process(const LidarObstacleDetectionOptions &options, LidarFrame *frame)
      {
        PointCloudPreprocessorOptions preprocessor_options;
        // 传感器转GPS的外参
        preprocessor_options.sensor2novatel_extrinsics = options.sensor2novatel_extrinsics;
        if (cloud_preprocessor_->Preprocess(preprocessor_options, frame))
        {
          return ProcessCommon(options, frame);
        }
        return LidarProcessResult(LidarErrorCode::PointCloudPreprocessorError,"Failed to preprocess point cloud.");
      }

      LidarProcessResult LidarObstacleDetection::Process(
          const LidarObstacleDetectionOptions &options,const std::shared_ptr<apollo::drivers::PointCloud const> &message, LidarFrame *frame)
      {
        const auto &sensor_name = options.sensor_name;
        // 用来屏蔽无效參数的,消除警告
        PERF_FUNCTION_WITH_INDICATOR(options.sensor_name);

        PERF_BLOCK_START();

        PointCloudPreprocessorOptions preprocessor_options;        // 传感器转GPS的外参 
        preprocessor_options.sensor2novatel_extrinsics = options.sensor2novatel_extrinsics;
        PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "preprocess");
        // 点云预处理
        if (cloud_preprocessor_->Preprocess(preprocessor_options, message, frame)) 
        {
          return ProcessCommon(options, frame);          // 模型推理
        }
        return LidarProcessResult(LidarErrorCode::PointCloudPreprocessorError,
                                  "Failed to preprocess point cloud.");
      }

      LidarProcessResult LidarObstacleDetection::ProcessCommon(
          const LidarObstacleDetectionOptions &options, LidarFrame *frame)
      {
        const auto &sensor_name = options.sensor_name;

        PERF_BLOCK_START();
        // 是否使用hdmap（高精地图）
        if (use_map_manager_)
        {
          MapManagerOptions map_manager_options; // 定义参数
          if (!map_manager_.Update(map_manager_options, frame)) // 启动..节点相关操作
          {
            return LidarProcessResult(LidarErrorCode::MapManagerError,"Failed to update map structure."); // 启动失败有错报错
          }
        }
        PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "map_manager");

        LidarDetectorOptions detection_options; // 定义障碍物检测参数
        if (!detector_->Detect(detection_options, frame))
        {
          return LidarProcessResult(LidarErrorCode::DetectionError,"Failed to detect.");
        }
        PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "detection");

        if (use_object_builder_)
        {
          ObjectBuilderOptions builder_options;
          if (!builder_.Build(builder_options, frame))
          {
            return LidarProcessResult(LidarErrorCode::ObjectBuilderError,"Failed to build objects.");
          }
        }
        PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "object_builder");

        if (use_object_filter_bank_)
        {
          ObjectFilterOptions filter_options; // 定义过滤参数
          if (!filter_bank_.Filter(filter_options, frame))
          {
            return LidarProcessResult(LidarErrorCode::ObjectFilterError,
                                      "Failed to filter objects.");
          }
        }
        PERF_BLOCK_END_WITH_INDICATOR(sensor_name, "filter_bank");

        return LidarProcessResult(LidarErrorCode::Succeed);
      }

      // 调用 apollo/modules/perception/lidar/lib/interface/base_lidar_obstacle_detection.h 定义的宏PERCEPTION_REGISTER_LIDAROBSTACLEDETECTION
      PERCEPTION_REGISTER_LIDAROBSTACLEDETECTION(LidarObstacleDetection);

    } // namespace lidar
  }   // namespace perception
} // namespace apollo
