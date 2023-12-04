/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.2
 * @Author: renjy
 * @Date: 2023-06-01 02:02:50
 * @LastEditTime: 2023-11-30 19:58:27
 */

#include "float.h"  // NOLINT

#include "reflector_mapping/submap.h"
#include "include/mapping_and_location_math.h"
#include "common/tool.h"


namespace gomros {
namespace data_process {
namespace mapping_and_location {

Submap::Submap(const MappingConfig& config,
  internal_common::ThreadPool* thread_pool)
: thread_pool_(thread_pool) {
  config_ = config;
  reflector_ekf_cal_ = std::make_shared<DataFusion::ReflectorEkfCalcuator>(
      config.ekf_config, config.sensor_mount, config.land_mark_config);
  Octree::Sides3 side(config_.land_mark_config.reflector_max_distance / 2.0,
                      config_.land_mark_config.reflector_max_distance / 2.0,
                      config_.land_mark_config.reflector_max_distance / 2.0);
  octree_reflector_map_ =
    std::make_shared<Octree::Map>(side, side);
  Eigen::Vector3d initial_pose = {0, 0, 0};
  Eigen::Matrix3d initial_cov;
  initial_cov.setZero();
  global_pose_ = {initial_pose, initial_cov};
  is_first_data_ = true;
  num_range_data_ = 0;
  submap_id_ = 0;
  min_x = FLT_MAX;
  min_y = FLT_MAX;
  max_x = -FLT_MAX;
  max_y = -FLT_MAX;
}

void Submap::InitReflectorEkf(const DataFusion::State& state) {
  SLAM_INFO("init state is (%f %f %f)", state.mu(0), state.mu(1), state.mu(2));
  is_first_data_ = false;
  last_state_ = state;
  global_pose_.first = state.mu.head(3);
  global_pose_.second = state.sigma.block(0, 0, 3, 3);
  finished_pose_ = global_pose_;
  DataFusion::State state_tmp;
  state_tmp.Init();
  state_tmp.time = state.time;
  state_tmp.mu(0) = state.mu(0);
  state_tmp.mu(1) = state.mu(1);
  state_tmp.mu(2) = state.mu(2);
  reflector_ekf_cal_->SetInitState(state_tmp);
}


void Submap::HandleOdomData(const OdometerMessage& data) {
  if (is_first_data_) {
    is_first_data_ = false;
    DataFusion::State state;
    state.Init();
    state.time = data.mclDeltaPosition.mlTimestamp;
    reflector_ekf_cal_->SetInitState(state);
    return;
  }
  reflector_ekf_cal_->HandleOdomData(data);
  return;
}

void Submap::HandleImuData(const ImuSensoryMessage& imu_data) {
  if (is_first_data_) {
    is_first_data_ = false;
    DataFusion::State state;
    state.Init();
    state.time = imu_data.time_stamp;
    reflector_ekf_cal_->SetInitState(state);
    return;
  }
  reflector_ekf_cal_->HandleImuData(imu_data);
}



int Submap::HandleObsData(const DataFusion::Observation& obs) {
  if (is_first_data_) {
    is_first_data_ = false;
    DataFusion::State state;
    state.Init();
    state.time = obs.time_;
    reflector_ekf_cal_->SetInitState(state);
    return 0;
  }

  reflector_ekf_cal_->HandleObsData(obs);
  DataFusion::State state;
  reflector_ekf_cal_->GetState(&state);
  SLAM_DEBUG("SubMapId %d range_data_count %d submap_state %f %f %f",
      submap_id_, num_range_data_++, state.mu(0), state.mu(1), state.mu(2));
  _calStateBox(state);
  if (num_range_data_ > config_.land_mark_config.max_ladar_size_in_sub_map) {
    SLAM_WARN("intert obs is out of control, range_data_count %d",
      num_range_data_);
  }
  if (num_range_data_ == config_.land_mark_config.max_ladar_size_in_sub_map) {
    SLAM_INFO("submap %d is can be finished~~", submap_id_);
  }
  return num_range_data_;
}

void Submap::FinishSubMap() {
  // 获取子图的相关信息
  DataFusion::State state;
  reflector_ekf_cal_->GetState(&state);
  finished_pose_.first = state.mu.head(3);
  finished_pose_.second = state.sigma.block(0, 0, 3, 3);
  int size = (state.mu.size() - 3) / 2;
  int last_size = (last_state_.mu.size() - 3) / 2;
  last_size = -1;
  double submap_global_x = global_pose_.first.x();
  double submap_global_y = global_pose_.first.y();
  double submap_global_theta = global_pose_.first.z();
  SLAM_INFO("SubMap id %d submap_box %f %f %f %f",
    submap_id_, min_x, min_y, max_x, max_y);
  SLAM_INFO("SubMap id %d sum_size %d, last_size %d",
             submap_id_, size, last_size);

  char buff[512];
  snprintf(buff, sizeof(buff), "submap_%02d.smap", submap_id_);
  std::string map_name = buff;
  Json::Value map_json;
  Json::Value landmark_point;
  Json::Value map_header;
  Json::Value atring_add;
  map_header["mapType"] = "Reflector-Map";
  map_header["mapName"] = map_name;
  int min_reflector_count_in_map =
    config_.land_mark_config.max_ladar_size_in_sub_map * 0.1;

  int land_mark_count = 0;
  for (int i = 0; i < size; i++) {
    int reflector_count = state.obs_in_map_count[i];
    if (i < last_size) {
      reflector_count -= (last_state_.obs_in_map_count[i] * 0.2);
    }
    if (reflector_count > min_reflector_count_in_map) {
      // TODO(r) 修改一下
      Eigen::Vector2d global_center(state.mu(3 + 2 * i),
                                    state.mu(3 + 2 * i + 1));
      Eigen::Matrix2d coviarance =
        state.sigma.block(3 + 2 * i, 3 + 2 * i, 2, 2);
      Eigen::Vector2d local_center;
      local_center.x() = (global_center(0) - submap_global_x) *
                          std::cos(submap_global_theta) +
                          (global_center(1) - submap_global_y) *
                          std::sin(submap_global_theta);
      local_center.y() = -(global_center(0) - submap_global_x) *
                          std::sin(submap_global_theta) +
                          (global_center(1) - submap_global_y) *
                          std::cos(submap_global_theta);
      reflector_in_local_.reflectors_.push_back(local_center);
      reflector_in_local_.reflector_map_coviarance_.push_back(coviarance);
      SLAM_INFO("submap_id %d in_global_pos %f %f "
                "there have %d obs_in_map_count %d",
                 submap_id_, global_center(0), global_center(1),
                 i, state.obs_in_map_count[i]);
      Json::Value landmark_info;
      landmark_info["x"] = local_center.x();
      landmark_info["y"] = local_center.y();
      landmark_info["radius"] = config_.land_mark_config.landmark_radius;
      landmark_info["landmark_id"] = land_mark_count;
      landmark_point[land_mark_count] = landmark_info;
      land_mark_count++;
    } else {
      SLAM_WARN("there have %d obs_in_map_count %d",
                i, state.obs_in_map_count[i]);
    }
  }
  atring_add["x"] = -100.0;
  atring_add["y"] = -100.0;
  map_header["minPos"] = atring_add;
  atring_add["x"] = 200.0;
  atring_add["y"] = 200.0;
  map_header["maxPos"] = atring_add;
  map_json["header"] = map_header;

  map_json["landmarks_pos_list"] = landmark_point;
  internal_common::Save(map_name.c_str(), map_json);
  // 存入八叉树
  _fillInOctree();
  return;
}

void Submap::_fillInOctree() {
  const std::vector<Eigen::Vector2d> reflector_in_local =
    reflector_in_local_.reflectors_;
  int reflectors_size = reflector_in_local.size();
  if (reflectors_size < 3) {
    SLAM_WARN("submap id %d reflectors size is %d,not enough",
              submap_id_, reflectors_size);
    return;
  } else {
    SLAM_INFO("submap id %d reflectors size is %d",
              submap_id_, reflectors_size);
  }
  // 计算三边边长
  float side_distance_ij, side_distance_ik, side_distance_jk;
  int real_triangle_count = 0;
  // 预计放入三角形个数
  int estimate_triangle_size =
    reflectors_size * (reflectors_size - 1) * (reflectors_size - 2) / 6;
  Octree::OctreePoint *octreePoints =
    new Octree::OctreePoint[estimate_triangle_size];

  for (int i = 0; i < reflector_in_local.size(); i++) {
    for (int j = i + 1; j < reflector_in_local.size(); j++) {
      if (i == j) continue;
      for (int k = j + 1; k < reflector_in_local.size(); k++) {
        if (k == i || k == j) continue;
        side_distance_ij = SLAMMath::Dist(reflector_in_local[i](0),
                                          reflector_in_local[i](1),
                                          reflector_in_local[j](0),
                                          reflector_in_local[j](1));
        side_distance_ik = SLAMMath::Dist(reflector_in_local[i](0),
                                          reflector_in_local[i](1),
                                          reflector_in_local[k](0),
                                          reflector_in_local[k](1));
        side_distance_jk = SLAMMath::Dist(reflector_in_local[j](0),
                                          reflector_in_local[j](1),
                                          reflector_in_local[k](0),
                                          reflector_in_local[k](1));
        // 判断是满足放入八叉树的条件
        if (!_checkTriangle(
            side_distance_ij, side_distance_ik, side_distance_jk))
            continue;
        Octree::Sides3 side(side_distance_ij,
                            side_distance_ik, side_distance_jk);
        std::vector<int> landmark_local_ids{i, j, k};
        octreePoints[real_triangle_count].setValue(side, landmark_local_ids);
        octree_reflector_map_->Insert(octreePoints + real_triangle_count);
        real_triangle_count++;
      }
    }
  }
}

bool Submap::_checkTriangle(float side_1, float side_2, float side_3) {
  float diff_threshold = config_.land_mark_config.distance_diff_threshold;
  float distance_threshold = config_.land_mark_config.reflector_max_distance;
  float theta_1 = std::acos(0.5 * (std::pow(side_1, 2) + std::pow(side_2, 2) -
                                   std::pow(side_3, 2)) / (side_1 * side_2));
  float theta_2 = std::acos(0.5 * (std::pow(side_1, 2) + std::pow(side_3, 2) -
                                    std::pow(side_2, 2)) / (side_1 * side_3));
  float theta_3 = std::acos(0.5 * (std::pow(side_2, 2) + std::pow(side_3, 2) -
                                    std::pow(side_1, 2)) / (side_2 * side_3));
  if (fabs(theta_1) < 0.04 || fabs(theta_2) < 0.04 || fabs(theta_3) < 0.04)
    return false;
  if (side_1 > distance_threshold) return false;
  if (side_2 > distance_threshold) return false;
  if (side_3 > distance_threshold) return false;
  if (fabs(side_1 - side_2) < diff_threshold) return false;
  if (fabs(side_1 - side_3) < diff_threshold) return false;
  if (fabs(side_2 - side_3) < diff_threshold) return false;
  return true;
}

void Submap::_calStateBox(const DataFusion::State& state) {
  if (state.mu(0) < min_x)  min_x = state.mu(0);
  if (state.mu(1) < min_y)  min_y = state.mu(1);
  if (state.mu(0) > max_x)  max_x = state.mu(0);
  if (state.mu(1) > max_y)  max_y = state.mu(1);
  return;
}



}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros



