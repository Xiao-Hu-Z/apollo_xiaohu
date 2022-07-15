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
#include "modules/perception/lidar/lib/detector/point_pillars_detection/point_pillars_detection.h"

#include <algorithm>
#include <numeric>
#include <random>

#include <cuda_runtime_api.h>

#include "cyber/common/log.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/base/point_cloud_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lidar/common/lidar_timer.h"
#include "modules/perception/lidar/common/pcl_util.h"
#include "modules/perception/lidar/lib/detector/point_pillars_detection/params.h"

namespace apollo {
namespace perception {
namespace lidar {

using base::Object;
using base::PointD;
using base::PointF;

PointPillarsDetection::PointPillarsDetection()
    : x_min_(Params::kMinXRange), // -74.88f
      x_max_(Params::kMaxXRange), // 74.88f
      y_min_(Params::kMinYRange), // -74.88f
      y_max_(Params::kMaxYRange), // 74.88f
      z_min_(Params::kMinZRange), // -2.0f
      z_max_(Params::kMaxZRange) { // 4.0f
  // 调用DEFINE_bool宏获取hdmap选择FLAGS_enable_ground_removal,默认false
  // DEFINE_bool宏位于：modules/perception/common/perception_gflags.h
  if (FLAGS_enable_ground_removal) {
    // FLAGS_ground_removal_height默认-1.5
    z_min_ = std::max(z_min_, static_cast<float>(FLAGS_ground_removal_height));
  }
}

// TODO(chenjiahao):
//  specify score threshold and nms over lap threshold for each class.
bool PointPillarsDetection::Init(const LidarDetectorInitOptions& options) {
  // FLAGS_reproduce_result_mode 默认false
  // FLAGS_score_threshold 默认0.5
  // FLAGS_nms_overlap_threshold 默认0.5
  // FLAGS_pfe_torch_file: /apollo/modules/perception/production/data/perception/lidar/models/detection/point_pillars/pts_voxel_encoder.zip
  // FLAGS_scattered_torch_file : /apollo/modules/perception/production/data/perception/lidar/models/detection/point_pillars/pts_middle_encoder.zip
  // backbone_torch_file : /apollo/modules/perception/production/data/perception/lidar/models/detection/point_pillars/pts_backbone.zip
  // FLAGS_fpn_torch_file : /apollo/modules/perception/production/data/perception/lidar/models/detection/point_pillars/pts_neck.zip
  // FLAGS_bbox_head_torch_file : /apollo/modules/perception/production/data/perception/lidar/models/detection/point_pillars/pts_bbox_head.zip
  point_pillars_ptr_.reset(
      new PointPillars(FLAGS_reproduce_result_mode, FLAGS_score_threshold,
                       FLAGS_nms_overlap_threshold, FLAGS_pfe_torch_file,
                       FLAGS_scattered_torch_file, FLAGS_backbone_torch_file,
                       FLAGS_fpn_torch_file, FLAGS_bbox_head_torch_file));
  return true;
}

bool PointPillarsDetection::Detect(const LidarDetectorOptions& options,
                                   LidarFrame* frame) {
  // check input
  if (frame == nullptr) {
    AERROR << "Input null frame ptr.";
    return false;
  }
  if (frame->cloud == nullptr) {
    AERROR << "Input null frame cloud.";
    return false;
  }
  if (frame->cloud->size() == 0) {
    AERROR << "Input none points.";
    return false;
  }

  // record input cloud and lidar frame
  original_cloud_ = frame->cloud;
  original_world_cloud_ = frame->world_cloud;
  lidar_frame_ref_ = frame;

  // check output
  frame->segmented_objects.clear();
  // FLAGS_gpu_id 默认为0，使用0号显卡
  if (cudaSetDevice(FLAGS_gpu_id) != cudaSuccess) {
    AERROR << "Failed to set device to gpu " << FLAGS_gpu_id;
    return false;
  }
  // 调用Tine构造函数，成员变量_start = std::chrono::system_clock::now(); // 获取系统的时间戳，单位微秒
  Timer timer;

  int num_points;
  cur_cloud_ptr_ = std::shared_ptr<base::PointFCloud>(new base::PointFCloud(*original_cloud_));

  // down sample the point cloud through filtering beams
  // FLAGS_enable_downsample_beams 默认 false
  if (FLAGS_enable_downsample_beams) {
    base::PointFCloudPtr downsample_beams_cloud_ptr(new base::PointFCloud());
    // 通过beam_id下采样，下采样因子FLAGS_downsample_beams_factor默认为4，减少点云数据量
    if (DownSamplePointCloudBeams(original_cloud_, downsample_beams_cloud_ptr,FLAGS_downsample_beams_factor)) {
      cur_cloud_ptr_ = downsample_beams_cloud_ptr;
    } else {
      AWARN << "Down-sample beams factor must be >= 1. Cancel down-sampling."
               " Current factor: "
            << FLAGS_downsample_beams_factor;
    }
  }

  // down sample the point cloud through filtering voxel grid
  // 体素滤波下采样，FLAGS_enable_downsample_pointcloud默认为false
  if (FLAGS_enable_downsample_pointcloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    TransformToPCLXYZI(*cur_cloud_ptr_, pcl_cloud_ptr);
    // 下采样尺寸xyz分为默认为0.01
    DownSampleCloudByVoxelGrid(pcl_cloud_ptr, filtered_cloud_ptr, FLAGS_downsample_voxel_size_x,FLAGS_downsample_voxel_size_y, FLAGS_downsample_voxel_size_z);

    // transform pcl point cloud to apollo point cloud
    base::PointFCloudPtr downsample_voxel_cloud_ptr(new base::PointFCloud());
    TransformFromPCLXYZI(filtered_cloud_ptr, downsample_voxel_cloud_ptr);
    cur_cloud_ptr_ = downsample_voxel_cloud_ptr;
  }
  // 计算下采样时间
  downsample_time_ = timer.toc(true);

  num_points = cur_cloud_ptr_->size();
  AINFO << "num points before fusing: " << num_points;

  // fuse clouds of preceding frames with current cloud
  // fuse 的是当前的点云和的点云，作用：当点云比较稀疏时，为了提升检测效果，一般会把当前帧的点云和前几帧点云融合，弥补稀疏效果
  // 点云成员变量points_timestamp_初始化
  cur_cloud_ptr_->mutable_points_timestamp()->assign(cur_cloud_ptr_->size(),0.0);
  if (FLAGS_enable_fuse_frames && FLAGS_num_fuse_frames > 1) {
    // before fusing
    // 将融合时间间隔大于0.5的点过滤
    while (!prev_world_clouds_.empty() && frame->timestamp - prev_world_clouds_.front()->get_timestamp() > FLAGS_fuse_time_interval) {
      prev_world_clouds_.pop_front();
    }
    // transform current cloud to world coordinate and save to a new ptr
    // 将当前帧的点云转到世界坐标系下，包含xyzi信息
    base::PointDCloudPtr cur_world_cloud_ptr = std::make_shared<base::PointDCloud>();
    for (size_t i = 0; i < cur_cloud_ptr_->size(); ++i) {
      auto& pt = cur_cloud_ptr_->at(i);
      Eigen::Vector3d trans_point(pt.x, pt.y, pt.z);
      trans_point = lidar_frame_ref_->lidar2world_pose * trans_point;
      PointD world_point;
      world_point.x = trans_point(0);
      world_point.y = trans_point(1);
      world_point.z = trans_point(2);
      world_point.intensity = pt.intensity;
      cur_world_cloud_ptr->push_back(world_point);
    }
    cur_world_cloud_ptr->set_timestamp(frame->timestamp);

    // fusing clouds
    for (auto& prev_world_cloud_ptr : prev_world_clouds_) {
      num_points += prev_world_cloud_ptr->size();
    }
    // 将过滤后的之前点云加入到当前帧点云中
    FuseCloud(cur_cloud_ptr_, prev_world_clouds_);

    // after fusing
    // FLAGS_num_fuse_frames 默认为5
    while (static_cast<int>(prev_world_clouds_.size()) >= FLAGS_num_fuse_frames - 1) {
      prev_world_clouds_.pop_front();
    }
    prev_world_clouds_.emplace_back(cur_world_cloud_ptr);
  }
  AINFO << "num points after fusing: " << num_points;
  // 计算fuse时间
  fuse_time_ = timer.toc(true);

  // shuffle points and cut off
  // enable_shuffle_points 默认false
  if (FLAGS_enable_shuffle_points) {
    // FLAGS_max_num_points 默认为int的最大值 2^31-1
    num_points = std::min(num_points, FLAGS_max_num_points);
    // 对[0-num_points)之间索引，打乱
    std::vector<int> point_indices = GenerateIndices(0, num_points, true);
    // 获取打乱后的点云
    base::PointFCloudPtr shuffle_cloud_ptr(new base::PointFCloud(*cur_cloud_ptr_, point_indices));
    cur_cloud_ptr_ = shuffle_cloud_ptr;
  }
  // 计算数据shuffle时间
  shuffle_time_ = timer.toc(true);

  // point cloud to array
  float* points_array = new float[num_points * FLAGS_num_point_feature]();
  // FLAGS_normalizing_factor :255
  // 过滤范围外的点云，将点云数据转为一维数组
  CloudToArray(cur_cloud_ptr_, points_array, FLAGS_normalizing_factor);
  cloud_to_array_time_ = timer.toc(true);

  // inference
  std::vector<float> out_detections;
  std::vector<int> out_labels;
  point_pillars_ptr_->DoInference(points_array, num_points, &out_detections,&out_labels);
  inference_time_ = timer.toc(true);

  // transfer output bounding boxes to objects
  // segmented_objects:每帧检测出的objects
  GetObjects(&frame->segmented_objects, frame->lidar2world_pose,&out_detections, &out_labels);
  collect_time_ = timer.toc(true);

  AINFO << "PointPillars: "
        << "\n"
        << "down sample: " << downsample_time_ << "\t"
        << "fuse: " << fuse_time_ << "\t"
        << "shuffle: " << shuffle_time_ << "\t"
        << "cloud_to_array: " << cloud_to_array_time_ << "\t"
        << "inference: " << inference_time_ << "\t"
        << "collect: " << collect_time_;

  delete[] points_array;
  return true;
}

void PointPillarsDetection::CloudToArray(const base::PointFCloudPtr& pc_ptr,
                                         float* out_points_array,
                                         const float normalizing_factor) {
  for (size_t i = 0; i < pc_ptr->size(); ++i) {
    const auto& point = pc_ptr->at(i);
    float x = point.x;
    float y = point.y;
    float z = point.z;
    float intensity = point.intensity;
    if (z < z_min_ || z > z_max_ || y < y_min_ || y > y_max_ || x < x_min_ ||
        x > x_max_) {
      continue;
    }
    out_points_array[i * FLAGS_num_point_feature + 0] = x;
    out_points_array[i * FLAGS_num_point_feature + 1] = y;
    out_points_array[i * FLAGS_num_point_feature + 2] = z;
    out_points_array[i * FLAGS_num_point_feature + 3] = intensity / normalizing_factor;
    // delta of timestamp between prev and cur frames
    // FLAGS_num_point_feature 为5
    out_points_array[i * FLAGS_num_point_feature + 4] = static_cast<float>(pc_ptr->points_timestamp(i));
  }
}

void PointPillarsDetection::FuseCloud(const base::PointFCloudPtr& out_cloud_ptr,const std::deque<base::PointDCloudPtr>& fuse_clouds) {
  for (auto iter = fuse_clouds.rbegin(); iter != fuse_clouds.rend(); ++iter) {
    double delta_t = lidar_frame_ref_->timestamp - (*iter)->get_timestamp();
    // transform prev world point cloud to current sensor's coordinates
    for (size_t i = 0; i < (*iter)->size(); ++i) {
      auto& point = (*iter)->at(i);
      Eigen::Vector3d trans_point(point.x, point.y, point.z);
      trans_point = lidar_frame_ref_->lidar2world_pose.inverse() * trans_point;
      base::PointF pt;
      pt.x = static_cast<float>(trans_point(0));
      pt.y = static_cast<float>(trans_point(1));
      pt.z = static_cast<float>(trans_point(2));
      pt.intensity = static_cast<float>(point.intensity);
      // delta of time between current and prev frame
    }
      out_cloud_ptr->push_back(pt, delta_t);
  }
}

std::vector<int> PointPillarsDetection::GenerateIndices(int start_index,int size,bool shuffle) {
  // create a range number array
  std::vector<int> indices(size);
  std::iota(indices.begin(), indices.end(), start_index);

  // shuffle the index array
  if (shuffle) {
    unsigned seed = 0;
    std::shuffle(indices.begin(), indices.end(),std::default_random_engine(seed));
  }
  return indices;
}

void PointPillarsDetection::GetObjects(
    std::vector<std::shared_ptr<Object>>* objects, const Eigen::Affine3d& pose,
    std::vector<float>* detections, std::vector<int>* labels) {
  // 目标个数
  int num_objects = detections->size() / FLAGS_num_output_box_feature; // FLAGS_num_output_box_feature 为 7

  objects->clear();
  // 结合单例设计模式，调用并发对象池，创建num_objects个Object对象
  base::ObjectPool::Instance().BatchGet(num_objects, objects);

  for (int i = 0; i < num_objects; ++i) {
    auto& object = objects->at(i);
    object->id = i;

    // read params of bounding box
    float x = detections->at(i * FLAGS_num_output_box_feature + 0);
    float y = detections->at(i * FLAGS_num_output_box_feature + 1);
    float z = detections->at(i * FLAGS_num_output_box_feature + 2);
    float dx = detections->at(i * FLAGS_num_output_box_feature + 4);
    float dy = detections->at(i * FLAGS_num_output_box_feature + 3);
    float dz = detections->at(i * FLAGS_num_output_box_feature + 5);
    float yaw = detections->at(i * FLAGS_num_output_box_feature + 6);
    // 获取预测的偏航角，范围在[-pi/2,pi/2]
    yaw += M_PI / 2;
    yaw = std::atan2(sinf(yaw), cosf(yaw));
    yaw = -yaw;

    // directions
    object->theta = yaw;
    object->direction[0] = cosf(yaw);
    object->direction[1] = sinf(yaw);
    object->direction[2] = 0;
    object->lidar_supplement.is_orientation_ready = true;

    // compute vertexes of bounding box and transform to world coordinate
    object->lidar_supplement.num_points_in_roi = 8;
    object->lidar_supplement.on_use = true;
    object->lidar_supplement.is_background = false;
    float roll = 0, pitch = 0;
    // 欧拉角转旋转向量
    Eigen::Quaternionf quater =
        Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f translation(x, y, z);
    // 计算放射变换矩阵，平移向量*旋转向量
    Eigen::Affine3f affine3f = translation * quater.toRotationMatrix();
    // 包围框的8个顶点坐标，8个顶点在世界坐标系下的位置
    for (float vx : std::vector<float>{dx / 2, -dx / 2}) {
      for (float vy : std::vector<float>{dy / 2, -dy / 2}) {
        for (float vz : std::vector<float>{0, dz}) { // PointPillarsDetection 推理输出：z为3d包围框的z方向的最小值最小值
          Eigen::Vector3f v3f(vx, vy, vz);
          // 每个顶点的放射变换矩阵
          v3f = affine3f * v3f;
          PointF point;
          point.x = v3f.x();
          point.y = v3f.y();
          point.z = v3f.z();
          object->lidar_supplement.cloud.push_back(point);

          Eigen::Vector3d trans_point(point.x, point.y, point.z);
          trans_point = pose * trans_point;
          PointD world_point;
          world_point.x = trans_point(0);
          world_point.y = trans_point(1);
          world_point.z = trans_point(2);
          object->lidar_supplement.cloud_world.push_back(world_point);
        }
      }
    }

    // classification
    // 枚举 MAX_OBJECT_TYPE = 6 表示大目标
    // raw_probs二维数组，表示每个分类方法的概率
    object->lidar_supplement.raw_probs.push_back(std::vector<float>(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0.f));
    // Name()返回"PointPillarsDetection" ，raw_classification_methods 存储检测方法的类名
    object->lidar_supplement.raw_classification_methods.push_back(Name());
    // 获取object的子类型，比较细的划分：CAR，PEDESTRIAN，CYCLIST，UNKNOWN
    object->sub_type = GetObjectSubType(labels->at(i));
    // 大类别，分的不是很细：VEHICLE，BICYCLE，UNKNOWN_MOVABLE
    object->type = base::kSubType2TypeMap.at(object->sub_type);
    object->lidar_supplement.raw_probs.back()[static_cast<int>(object->type)] = 1.0f;
    // 每个类别的概率存储在type_probs字段
    object->type_probs.assign(object->lidar_supplement.raw_probs.back().begin(),object->lidar_supplement.raw_probs.back().end());
  }
}

// TODO(all): update the base ObjectSubType with more fine-grained types
// TODO(chenjiahao): move types into an array in the same order as offline
base::ObjectSubType PointPillarsDetection::GetObjectSubType(const int label) {
  switch (label) {
    case 0:
      return base::ObjectSubType::CAR;
    case 1:
      return base::ObjectSubType::PEDESTRIAN;
    case 2:  // construction vehicle
      return base::ObjectSubType::CYCLIST;
    default:
      return base::ObjectSubType::UNKNOWN;
  }
}

PERCEPTION_REGISTER_LIDARDETECTOR(PointPillarsDetection);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
