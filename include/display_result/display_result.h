/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-05-20 02:23:21
 * @LastEditors: renjy
 * @LastEditTime: 2023-11-18 14:35:20
 */
#pragma once
#include <vector>

#include "include/config_struct.h"
#include "message_lib/radar_message.h"
#include "message_lib/odometer_message.h"
#include "message_lib/simple_grid_map.h"

#include "common_lib/node.h"

namespace DisPlayResult {

using namespace gomros::data_process::mapping_and_location;  // NOLINT


class SimulationDataPublisher {
 public:
  using Node = gomros::common::Node;
  using RawPublisher = gomros::common::RawPublisher;
  using RadarSensoryInfo = gomros::message::RadarSensoryInfo;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using Position = gomros::message::Position;
  using SimpleGridMap = gomros::message::SimpleGridMap;

 public:
  explicit SimulationDataPublisher(const SensorMount& sensor_mount);
  virtual ~SimulationDataPublisher();

  /**
   * @name: 显示激光信息
   * @param message 激光点云信息 (原始)
   * @param current_pos 当前机器全局坐标
   * @return {*}
   */
  void DisplayLadarMessage(const RadarSensoryInfo& message);
  /**
   * @name: 显示激光信息及匹配的反光板信息
   * @param message 激光点云信息 （原始）具有雷达的全局位姿
   * @param obs 观测到的反光板信息 - 车体坐标系
   * @param match_obs 与地图匹配 -- 全局坐标系
   * @param cov 协方差矩阵
   * @return {*}
   */
  void DisPlayReflectorLocation(const RadarSensoryInfo& message,
                          const std::vector<Eigen::Vector2d>& obs,
                          const std::vector<Eigen::Vector2d>& match_obs,
                          const Eigen::Matrix2d& cov = Eigen::Matrix2d::Zero());

  /**
   * @name: 显示栅格地图
   * @param grid_map 栅格地图信息
   * @return
   */
  void DisPlayGridMap(SimpleGridMap& grid_map);  // NOLINT

  /**
   * @description: 为可视化反光柱建图
   * @param message 雷达信息 -- 反光柱信息 -- 已经建图信息
   * @return {*}
   */  
  void DisPlayReflectorMapping(const RadarSensoryInfo& raw_data,
                               const Eigen::VectorXd& state,
                               const std::vector<Eigen::Vector2d>& obs,
                               int map_id);



 private:
  SensorMount sensor_mount_;
  Node* node_;
  RawPublisher* publish_ladar_for_display_;
  RawPublisher* grid_map_publisher_;
  RawPublisher* reflector_mapping_publisher_;
  RawPublisher* reflector_Location_publisher_;
};


}  // namespace DisPlayResult

