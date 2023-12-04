/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-05-20 02:39:17
 * @LastEditors: renjy
 * @LastEditTime: 2023-11-18 14:44:37
 */
#include "display_result/display_result.h"
#include "display_result/display_topic.h"



namespace DisPlayResult {

SimulationDataPublisher::SimulationDataPublisher(
  const SensorMount& sensor_mount)
  : sensor_mount_(sensor_mount) {
  node_ = new Node("display");
  publish_ladar_for_display_ = node_->AdvertiseTopic(raw_ladar_topic);
  grid_map_publisher_ = node_->AdvertiseTopic(grid_map_topic);
  reflector_mapping_publisher_ = node_->AdvertiseTopic(reflector_mapping_topic);
  reflector_Location_publisher_ = node_->AdvertiseTopic(reflector_ladar_topic);
}

SimulationDataPublisher::~SimulationDataPublisher() {}

void SimulationDataPublisher::DisplayLadarMessage(
  const RadarSensoryInfo& message) {
  RadarSensoryMessage message_result;
  message_result.mstruRadarMessage = message;
  std::vector<char> send_buff;
  message_result.to_char_array(&send_buff);
  publish_ladar_for_display_->Publish(send_buff.data(), send_buff.size());
}

void SimulationDataPublisher::DisPlayReflectorLocation(
  const RadarSensoryInfo& raw_ladar,
  const std::vector<Eigen::Vector2d>& obs,
  const std::vector<Eigen::Vector2d>& match_obs,
  const Eigen::Matrix2d& cov) {
  RadarSensoryMessage message;
  message.mstruRadarMessage = raw_ladar;
  float x = raw_ladar.mstruRadarHeaderData.mfXPos;
  float y = raw_ladar.mstruRadarHeaderData.mfYPos;
  float t = raw_ladar.mstruRadarHeaderData.mfTheta;
  for (auto iter : obs) {
    // 识别到的反光柱 -- 转至全局坐标下
    pcl::PointXYZI temp;
    temp.x = iter(0) * std::cos(t) - iter(1) * std::sin(t) + x;
    temp.y = iter(0) * std::sin(t) + iter(1) * std::cos(t) + y;
    temp.z = DisPlayResult::ReflectorType::Identify;
    message.mstruRadarMessage.mstruMultiLayerData.push_back(temp);
  }

  for (auto iter : match_obs) {
    // 地图中反光柱 -- 被匹配
    pcl::PointXYZI temp;
    temp.x = iter(0);
    temp.y = iter(1);
    temp.z = DisPlayResult::ReflectorType::MatchInMap;
    message.mstruRadarMessage.mstruMultiLayerData.push_back(temp);
  }
  std::vector<char> send_buff;
  message.to_char_array(&send_buff);
  reflector_Location_publisher_->Publish(send_buff.data(), send_buff.size());
  return;
}

void SimulationDataPublisher::DisPlayGridMap(SimpleGridMap& grid_map) {
  std::vector<char> send_buff;
  grid_map.to_json_char_array(&send_buff);
  grid_map_publisher_->Publish(send_buff.data(), send_buff.size());
}


void SimulationDataPublisher::DisPlayReflectorMapping(
  const RadarSensoryInfo& raw_data,
  const Eigen::VectorXd& state, const std::vector<Eigen::Vector2d>& obs,
  int map_id) {
  RadarSensoryMessage message;
  message.mstruRadarMessage = raw_data;

  message.mstruRadarMessage.mstruRadarHeaderData.mcPosValid = 1;
  message.mstruRadarMessage.mstruRadarHeaderData.mfXPos = state[0];
  message.mstruRadarMessage.mstruRadarHeaderData.mfYPos = state[1];
  message.mstruRadarMessage.mstruRadarHeaderData.mfTheta = state[2];
  message.mstruRadarMessage.mstruRadarHeaderData.mnLayerNum = map_id;

  const int state_obs_size = (state.rows() - 3) / 2;
  for (int j = 0; j < state_obs_size; ++j) {
    // 状态空间中第j个反光板
    pcl::PointXYZI temp;
    temp.x = state(3 + 2 * j);
    temp.y = state(3 + 2 * j + 1);
    temp.z = ReflectorType::InState;
    message.mstruRadarMessage.mstruMultiLayerData.push_back(temp);
  }
  for (int j = 0; j < obs.size(); ++j) {
    // 识别到的反光柱
    pcl::PointXYZI temp;
    temp.x = obs[j](0);
    temp.y = obs[j](1);
    temp.z = ReflectorType::Identify;
    message.mstruRadarMessage.mstruMultiLayerData.push_back(temp);
  }


  std::vector<char> send_buff;
  message.to_char_array(&send_buff);

  reflector_mapping_publisher_->Publish(send_buff.data(), send_buff.size());
}



}  // namespace DisPlayResult

