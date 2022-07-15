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
#include "modules/perception/onboard/component/detection_component.h"

#include "cyber/time/clock.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/lidar/common/lidar_error_code.h"
#include "modules/perception/lidar/common/lidar_frame_pool.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/onboard/common_flags/common_flags.h"

using ::apollo::cyber::Clock;

namespace apollo
{
  namespace perception
  {
    namespace onboard
    {
      // 原子数据
      std::atomic<uint32_t> DetectionComponent::seq_num_{0};
      bool DetectionComponent::Init()
      {
        // 读取配置文件，配置文件定义：modules/perception/onboard/proto/lidar_component_config.proto
        // lauunch文件启动，launch含有DAG，DAG文件定义模块的依赖关系，包含组件启动的配置参数，如name,config_file_path，flag_file_path等
        /*
        * config_file_path配置文件参数：Apollo/modules/perception/production/conf/perception/lidar/velodyne128_detection_conf.pb.txt
        * sensor_name: "velodyne128"
        * enable_hdmap: true
        * lidar_query_tf_offset: 0
        * lidar2novatel_tf2_child_frame_id: "velodyne128"
        * output_channel_name: "/perception/inner/DetectionObjects"
        */
        LidarDetectionComponentConfig comp_config;
        // 读取config_file_path信息存储到comp_config中
        if (!GetProtoConfig(&comp_config))
        {
          return false;
        }
        ADEBUG << "Lidar Component Configs: " << comp_config.DebugString(); // DebugString：打印输出comp_config对象
        output_channel_name_ = comp_config.output_channel_name(); // 发布通道名
        sensor_name_ = comp_config.sensor_name();                 // 传感器名
        detector_name_ = comp_config.detector_name();             // 检测算法类名LidarObstacleDetection        
        lidar2novatel_tf2_child_frame_id_ = comp_config.lidar2novatel_tf2_child_frame_id(); // 
        lidar_query_tf_offset_ = static_cast<float>(comp_config.lidar_query_tf_offset()); // lidar时间戳偏移
        enable_hdmap_ = comp_config.enable_hdmap();               // 是否启用高精度地图
        writer_ = node_->CreateWriter<LidarFrameMessage>(output_channel_name_);   // 类似于ROS发布tpoic,相当于初始化，

        // 初始化成员算法类
        if (!InitAlgorithmPlugin())
        {
          AERROR << "Failed to init detection component algorithm plugin.";
          return false;
        }
        return true;
      }

      bool DetectionComponent::Proc(const std::shared_ptr<drivers::PointCloud> &message)
      {
        // 以16位精度输出点云测量时间和当前时间，单位秒
        AINFO << std::setprecision(16)
              << "Enter detection component, message timestamp: "
              << message->measurement_time()
              << " current timestamp: " << Clock::NowInSeconds();

        auto out_message = std::make_shared<LidarFrameMessage>();

        bool status = InternalProc(message, out_message);
        if (status)
        {
          // 发布处理后数据
          writer_->Write(out_message);
          AINFO << "Send lidar detect output message.";
        }
        return status;
      }

      bool DetectionComponent::InitAlgorithmPlugin()
      {
        // 读取传感器元数据，元数据的读取是通过SensorManager来完成的,SensorManager 类经宏定义 DECLARE_SINGLETON(SensorManager) 修饰成为单例类，单例对象调用GetSensorInfo函数获取传感器名sensor_name_对应的传感器信息SensorInfo
        // 其在初始化时会读取modules/perception/production/data/perception/common/sensor_meta.pt的包含所有传感器元数据的列表
        ACHECK(common::SensorManager::Instance()->GetSensorInfo(sensor_name_,&sensor_info_));
        // apollo/modules/perception/lib/registerer/registerer.h
        // 父类指针detector指向子类LidarObstacleDetection的对象
        lidar::BaseLidarObstacleDetection *detector = lidar::BaseLidarObstacleDetectionRegisterer::GetInstanceByName(detector_name_); // 工厂模式,调用宏定义类的静态方法获取类LidarObstacleDetection的实例指针
        CHECK_NOTNULL(detector);
        detector_.reset(detector);
        // lidar型号，hdmap是否使用
        lidar::LidarObstacleDetectionInitOptions init_options;
        init_options.sensor_name = sensor_name_;
        // 调用DEFINE_bool宏返回FLAGS_obs_enable_hdmap_input,bool类型,表达是否有hdmap输入
        // DEFINE_bool宏位于：modules/perception/onboard/common_flags/common_flags.cpp
        init_options.enable_hdmap_input = FLAGS_obs_enable_hdmap_input && enable_hdmap_;
        // 多态性：子类LidarObstacleDetection重写父类BaseLidarObstacleDetection的虚函数init
        // 调用子类LidarObstacleDetection的init函数
        ACHECK(detector_->Init(init_options)) << "lidar obstacle detection init error";
        lidar2world_trans_.Init(lidar2novatel_tf2_child_frame_id_);
        return true;
      }
      

      bool DetectionComponent::InternalProc(
          const std::shared_ptr<const drivers::PointCloud> &in_message,
          const std::shared_ptr<LidarFrameMessage> &out_message)
      /*
      输入：drivers::PointCloud 原始点云
      输出：LidarFrameMessaglidar 处理结果
      */
      { // 序列号
        uint32_t seq_num = seq_num_.fetch_add(1);
        // 时间戳
        const double timestamp = in_message->measurement_time();
        // 当前时间
        const double cur_time = Clock::NowInSeconds();
        const double start_latency = (cur_time - timestamp) * 1e3;
        AINFO << std::setprecision(16) << "FRAME_STATISTICS:Lidar:Start:msg_time["
              << timestamp << "]:sensor[" << sensor_name_ << "]:cur_time[" << cur_time
              << "]:cur_latency[" << start_latency << "]";

        out_message->timestamp_ = timestamp;
        out_message->lidar_timestamp_ = in_message->header().lidar_timestamp();
        out_message->seq_num_ = seq_num;
        // 处理状态：检测
        out_message->process_stage_ = ProcessStage::LIDAR_DETECTION;
        // 错误码
        out_message->error_code_ = apollo::common::ErrorCode::OK;

        auto &frame = out_message->lidar_frame_;
        // 并发对象池，结合单例模式，获取目标的智能指针
        // 思想：一个对象只能有一个池子,用对象从池子里面取,每个池子有一个管理者来管理所对应的池子,取对象从管理者这里申请
        frame = lidar::LidarFramePool::Instance().Get();
        frame->cloud = base::PointFCloudPool::Instance().Get();
        frame->timestamp = timestamp;
        frame->sensor_info = sensor_info_;

        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        Eigen::Affine3d pose_novatel = Eigen::Affine3d::Identity();
        const double lidar_query_tf_timestamp = timestamp - lidar_query_tf_offset_ * 0.001;
        // 获取当前帧 坐标系到世界坐标系的位置变换，GPS到世界坐标系的位置变换
        // transform_wrapper原理和ROS tf变换一致，首先各传感器的位置关系需要通过广播形式发送出去
        if (!lidar2world_trans_.GetSensor2worldTrans(lidar_query_tf_timestamp, &pose, &pose_novatel))
        {
          out_message->error_code_ = apollo::common::ErrorCode::PERCEPTION_ERROR_TF;
          AERROR << "Failed to get pose at time: " << lidar_query_tf_timestamp;
          return false;
        }
        frame->lidar2world_pose = pose;
        frame->novatel2world_pose = pose_novatel;

        // 传感器名和lidar传感器转GPS的外参转换矩阵
        lidar::LidarObstacleDetectionOptions detect_opts;
        detect_opts.sensor_name = sensor_name_;
        // lidar到世界坐标系的变换 *lidar2world_trans_  =  *detect_opts.sensor2novatel_extrinsics
        lidar2world_trans_.GetExtrinsics(&detect_opts.sensor2novatel_extrinsics);
        // 掉用LidarObstacleDetection类的Process方法
        // frame.get() 获取存储的指针
        lidar::LidarProcessResult ret = detector_->Process(detect_opts, in_message, frame.get());
        if (ret.error_code != lidar::LidarErrorCode::Succeed)
        {
          out_message->error_code_ =
              apollo::common::ErrorCode::PERCEPTION_ERROR_PROCESS;
          AERROR << "Lidar detection process error, " << ret.log;
          return false;
        }

        return true;
      }

    } // namespace onboard
  }   // namespace perception
} // namespace apollo
