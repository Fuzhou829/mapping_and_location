/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-11-07 14:46:41
 * @LastEditors: renjy
 * @LastEditTime: 2023-11-30 12:21:58
 */
#include <gtest/gtest.h>
#include <memory>
#include <string>

#include <pcl/io/ply_io.h>  // NOLINT
#include <pcl/io/pcd_io.h>  // NOLINT
#include <pcl/point_types.h>  // NOLINT
#include <pcl/registration/icp.h>  // NOLINT
#include <pcl/visualization/pcl_visualizer.h>  // NOLINT

#include "mock/sensor_data_simulation.h"
#include "common/load_config.h"

#include "include/config_struct.h"
#include "mapping_and_location/mapping_and_location.h"
#include "mock/display_location_result.h"
#include "landmark_tool/trilateration.h"
#include "common/logger.h"


TEST(Reflector, location) {
  using namespace gomros::data_process::mapping_and_location;  // NOLINT
  LandMarkConfig land_mark_config;
  land_mark_config.error_threshold_for_loop = 0.1;
  land_mark_config.data_association_max_dis = 20;
  land_mark_config.reflector_max_distance = 40;
  land_mark_config.distance_diff_threshold = 1.0;
  LandMark::Trilateration trilateration_find_sim(land_mark_config);
  std::map<int, std::map<int, Eigen::Vector2d>> maps;
  std::map<int, Eigen::Vector2d> sub_map;
  sub_map.insert(std::make_pair(0, Eigen::Vector2d(-2.618979, -4.522776)));
  sub_map.insert(std::make_pair(1, Eigen::Vector2d(9.326328, -5.717954)));
  sub_map.insert(std::make_pair(2, Eigen::Vector2d(5.861104, 16.933195)));
  sub_map.insert(std::make_pair(3, Eigen::Vector2d(-2.353654, 3.685052)));
  sub_map.insert(std::make_pair(4, Eigen::Vector2d(-12.398353, 9.656034)));
  // sub_map.insert(std::make_pair(5, Eigen::Vector2d(25.211662, -5.326333)));
  // sub_map.insert(std::make_pair(6, Eigen::Vector2d(18.579738, -19.420214)));
  // sub_map.insert(std::make_pair(7, Eigen::Vector2d(6.571184, -19.540600)));
  // sub_map.insert(std::make_pair(8, Eigen::Vector2d(-0.203405, -19.098604)));
  // sub_map.insert(std::make_pair(9, Eigen::Vector2d(-5.228383, -15.111603)));
  // sub_map.insert(std::make_pair(10, Eigen::Vector2d(12.564900, 2.727135)));
  // sub_map.insert(std::make_pair(11, Eigen::Vector2d(0.536588, 2.596480)));
  // sub_map.insert(std::make_pair(12, Eigen::Vector2d(-11.685546, 2.460217)));
  // sub_map.insert(std::make_pair(13, Eigen::Vector2d(-0.250870, -15.072856)));
  pcl::PointCloud<pcl::PointXYZI>::Ptr
    cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
  float r = 0.375;
  for (auto iter : sub_map) {
      for (float i = -M_PI; i < M_PI; i += (M_PI / 180.f)) {
      pcl::PointXYZI point;
      point.x = iter.second(0) + r * std::cos(i);
      point.y = iter.second(1) + r * std::sin(i);;
      point.z = 1.0;
      point.intensity = 3000;
      cloud_in->push_back(point);
    }
  }

  if (!cloud_in->empty()) {
    pcl::io::savePCDFileASCII<pcl::PointXYZI> ("cloud_map.pcd", *cloud_in);
  }


  maps.insert(std::make_pair(-1, sub_map));
  cloud_in->clear();
  trilateration_find_sim.SetLandMarkMap(maps);
  std::vector<Eigen::Vector2d> obs;
  obs.push_back(Eigen::Vector2d(0.023468, -4.625286));
  obs.push_back(Eigen::Vector2d(-12.005385, -5.350520));
  obs.push_back(Eigen::Vector2d(7.676359, 17.150783));
  obs.push_back(Eigen::Vector2d(-10.315331, 9.164573));
  for (auto iter : obs) {
    for (float i = -M_PI; i < M_PI; i += (M_PI / 180.f)) {
      pcl::PointXYZI point;
      point.x = iter(0) + r * std::cos(i);
      point.y = iter(1) + r * std::sin(i);;
      point.z = 1.0;
      point.intensity = 10;
      cloud_in->push_back(point);
    }
  }

  if (!cloud_in->empty()) {
    pcl::io::savePCDFileASCII<pcl::PointXYZI> ("cloud_obs.pcd", *cloud_in);
  }
  // obs.push_back(Eigen::Vector2d(1.033018, -10.338236));
  // obs.push_back(Eigen::Vector2d(11.210511, -3.949097));
  // obs.push_back(Eigen::Vector2d(16.485418, 0.892622));

  Eigen::Vector3d global_pos;
  if (!trilateration_find_sim.IsGetGlobalPos(obs, &global_pos)) {
    SLAM_ERROR("error!!!!!!!!!!");
    return;
  }
  std::map<int, int> result;
  trilateration_find_sim.GetMatchId(&result);
  for (auto iter : result) {
    SLAM_INFO("map %d %f %f -> obs %d %f %f",
      iter.first, sub_map.at(iter.first)(0), sub_map.at(iter.first)(1),
      iter.second, obs[iter.second][0], obs[iter.second][1]);
  }
  for (auto iter1 : result) {
    for (auto iter2 : result) {
      if (iter1.first == iter2.first) continue;
      float dis_map = SLAMMath::Dist(
                      sub_map.at(iter1.first)(0), sub_map.at(iter1.first)(1),
                      sub_map.at(iter2.first)(0), sub_map.at(iter2.first)(1));
      float dis_obs = SLAMMath::Dist(
                      obs[iter1.second][0], obs[iter1.second][1],
                      obs[iter2.second][0], obs[iter2.second][1]);
      SLAM_INFO("match id %d %d -> %d %d distance map %f obs %f delta %f",
                iter1.first, iter2.first, iter1.second, iter2.second,
                dis_map, dis_obs, dis_map - dis_obs);
    }
  }


  SLAM_INFO("global_pos %f %f %f", global_pos[0], global_pos[1], global_pos[2]);
  // 转换至全局坐标系
  float c = cos(global_pos[2]);
  float s = sin(global_pos[2]);

  for (int i = 0; i < obs.size(); i++) {
    float x_w = obs[i][0] * c - obs[i][1] * s + global_pos[0];
    float y_w = obs[i][0] * s + obs[i][1] * c + global_pos[1];
    SLAM_INFO("obs id %d %f %f inwold %f %f",
                i, obs[i][0], obs[i][1], x_w, y_w);
  }

  return;
}



