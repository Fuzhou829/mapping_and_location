/*
 * @Descripttion: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @version: 1.0
 * @Author: renjy
 * @Date: 2023-03-15 15:04:58
 * @LastEditors: renjy
 * @LastEditTime: 2023-11-30 21:10:12
 */

#include <unistd.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include <pcl/io/ply_io.h>  // NOLINT
#include <pcl/io/pcd_io.h>  // NOLINT
#include <pcl/point_types.h>  // NOLINT
#include <pcl/registration/icp.h>  // NOLINT
#include <pcl/visualization/pcl_visualizer.h>  // NOLINT

#include "Eigen/Eigen"

#include "mock/display_location_result.h"
#include "mock/bresenham_line.h"
#include "jsoncpp/json/json.h"
#include "common/logger.h"
#include "display_result/display_topic.h"
#include "common/transform.h"


namespace Display {
// static cv::Mat show_mat_;

LocationResult::LocationResult(
  const mapping_and_location::MappingAndLocationConfig& config) {
  pthread_mutex_init(&raw_ladar_data_mutex_, nullptr);
  pthread_mutex_init(&picture_data_mutex_, nullptr);
  pthread_mutex_init(&reflector_mapping_mutex_, nullptr);
  pthread_mutex_init(&reflector_location_mutex_, nullptr);

  config_ = config;
  is_get_map_ = false;
  is_get_pos_ = false;

  // 加载地图用于可视化
  std::string file_dir =
    config_.mapping_config.map_data_file_path + "/" + "ale.smap";
  display_count_ = 0;
  if (!_loadMap(file_dir)) {
    SLAM_ERROR("can not load map !!!");
    return;
  }
  SLAM_INFO("load map successful!!!!!!!!!!!!!");
  // 订阅建图及定位结果
  node_ = new Node("display_node");
  CallBackEvent location_pose_event{this, _locationPosEvent};
  node_->SubscribeTopic(config_.ad_pose_topic, location_pose_event);

  CallBackEvent grid_map_event{this, _gridMapEvent};
  node_->SubscribeTopic(DisPlayResult::grid_map_topic, grid_map_event);


  pthread_create(&display_map_pthread, nullptr,
                 _displayMapPthread, this);

  CallBackEvent raw_ladar_event{this, _rawLadarEvent};
  node_->SubscribeTopic(DisPlayResult::raw_ladar_topic, raw_ladar_event);

  pthread_create(&display_ladar_pthread, nullptr,
                 _displayLadarInMapPthread, this);

  CallBackEvent reflector_mapping{this, _ReflectorMappingEvent};
  node_->SubscribeTopic(DisPlayResult::reflector_mapping_topic,
        reflector_mapping);

  CallBackEvent reflector_location{this, _ReflectorLocationEvent};
  node_->SubscribeTopic(DisPlayResult::reflector_ladar_topic,
        reflector_location);


  pthread_create(&display_reflector_mapping_pthread, nullptr,
                 _displayReflectorMappingPthread, this);

  pthread_create(&display_reflector_mapping_pthread, nullptr,
                 _displayReflectorLocationPthread, this);
}

LocationResult::~LocationResult() {
  node_->UnsubscribeTopic(config_.ad_pose_topic);
  node_->UnsubscribeTopic(DisPlayResult::grid_map_topic);
  node_->UnsubscribeTopic(DisPlayResult::raw_ladar_topic);
  node_->UnsubscribeTopic(DisPlayResult::reflector_ladar_topic);
  node_->UnsubscribeTopic(DisPlayResult::reflector_ekf_topic);
  delete node_;
}


bool LocationResult::IsFinishedDisply() {
  return false;
  return raw_ladar_datas_.empty() &&
         reflector_location_datas_.empty() &&
         reflector_mapping_datas_.empty();
}

bool LocationResult::_loadMap(const std::string& map_dir) {
  std::string map_data;
  if (!internal_common::Read(map_dir.c_str(), &map_data)) {
    SLAM_ERROR("read map failed.....");
    return false;
  }

  Json::Reader reader;
  Json::Value map_json;
  if (!reader.parse(map_data, map_json)) {
    SLAM_ERROR("parse map failed.....\n");
    return false;
  }
  pthread_mutex_lock(&picture_data_mutex_);
  map_info_.resolution = map_json["header"]["resolution"].asDouble();
  map_info_.origen_x = map_json["header"]["minPos"]["x"].asDouble();
  map_info_.origen_y = map_json["header"]["minPos"]["y"].asDouble();

  map_info_.width = static_cast<int>(ceil(
    (map_json["header"]["maxPos"]["x"].asDouble() -
    map_info_.origen_x) / map_info_.resolution));
  map_info_.height = static_cast<int>(ceil(
    (map_json["header"]["maxPos"]["y"].asDouble() -
    map_info_.origen_y) / map_info_.resolution));

  SLAM_INFO("location map info %d %d resolution %f",
            map_info_.width, map_info_.height, map_info_.resolution);
  show_mat_ = cv::Mat(cv::Size(map_info_.width, map_info_.height), CV_8UC3,
                            cv::Scalar(255, 255, 255));
  pthread_mutex_unlock(&picture_data_mutex_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr
    cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
  for (int j = 0; j < map_json["normalPosList"].size(); j++) {
    float x = map_json["normalPosList"][j]["x"].asDouble();
    float y = map_json["normalPosList"][j]["y"].asDouble();
    pcl::PointXYZI point;
    point.x = x;
    point.y = y;
    point.z = 0.0;
    point.intensity = 100;
    cloud_in->push_back(point);
    int grid_x, grid_y;
    if (!_world2Grid(x, y, &grid_x, &grid_y)) continue;
    pthread_mutex_lock(&picture_data_mutex_);
    show_mat_.at<cv::Vec3b>(grid_y, grid_x)[0] = 0;
    show_mat_.at<cv::Vec3b>(grid_y, grid_x)[1] = 0;
    show_mat_.at<cv::Vec3b>(grid_y, grid_x)[2] = 0;
    pthread_mutex_unlock(&picture_data_mutex_);
  }

  // 显示landmark
  float r = config_.mapping_config.land_mark_config.landmark_radius;
  for (int i = 0; i < map_json["landmarks_pos_list"].size(); i++) {
    float x = map_json["landmarks_pos_list"][i]["x"].asDouble();
    float y = map_json["landmarks_pos_list"][i]["y"].asDouble();
    for (float i = -M_PI; i < M_PI; i += (M_PI / 180.f)) {
      pcl::PointXYZI point;
      point.x = x + r * std::cos(i);
      point.y = y + r * std::sin(i);;
      point.z = 1.0;
      point.intensity = 3000;
      cloud_in->push_back(point);
    }
    int grid_x, grid_y;
    if (!_world2Grid(x, y, &grid_x, &grid_y)) continue;
    landmark_map_.push_back(LandMarkGrid(grid_x, grid_y));
    _loadLandMarkMap(grid_x, grid_y);
  }

  if (!cloud_in->empty()) {
    pcl::io::savePCDFileASCII<pcl::PointXYZI> ("cloud_in.pcd", *cloud_in);
  }
  // 显示qr
  for (int i = 0; i < map_json["qr_pos_list"].size(); i++) {
    float x = map_json["qr_pos_list"][i]["x"].asDouble();
    float y = map_json["qr_pos_list"][i]["y"].asDouble();
    int grid_x, grid_y;
    if (!_world2Grid(x, y, &grid_x, &grid_y)) continue;
    _loadQrMap(grid_x, grid_y);
  }

  return true;
}

void LocationResult::_changeMapInfo(float x, float y) {
  pthread_mutex_lock(&picture_data_mutex_);
  map_info_.width = 2500;
  map_info_.height = 2500;
  map_info_.origen_x = x - 25.0;
  map_info_.origen_y = y - 25.0;
  show_mat_ = cv::Mat(cv::Size(map_info_.width, map_info_.height), CV_8UC3,
                            cv::Scalar(255, 255, 255));
  pthread_mutex_unlock(&picture_data_mutex_);
}


void LocationResult::_loadLandMarkMap(int grid_x, int grid_y) {
  // 显示为红色的圆圈
  for (int xx = -3; xx <= 3; xx++) {
    for (int yy = -3; yy <= 3; yy++) {
      if (abs(xx) + abs(yy) != 3) continue;
      if (!_isInPicture(grid_x + xx, grid_y + yy)) continue;
      pthread_mutex_lock(&picture_data_mutex_);
      cv::circle(show_mat_, cv::Point2f(grid_x, grid_y), 5,
                  cv::Scalar(0, 0, 255), -1);
      pthread_mutex_unlock(&picture_data_mutex_);
    }
  }
}

void LocationResult::_loadQrMap(int grid_x, int grid_y) {
  // 显示为绿色的方块
  for (int xx = -3; xx <= 3; xx++) {
    for (int yy = -3; yy <= 3; yy++) {
      if (!_isInPicture(grid_x + xx, grid_y + yy)) continue;
      pthread_mutex_lock(&picture_data_mutex_);
      show_mat_.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[0] = 0;
      show_mat_.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[1] = 255;
      show_mat_.at<cv::Vec3b>(grid_y + yy, grid_x + xx)[2] = 0;
      pthread_mutex_unlock(&picture_data_mutex_);
    }
  }
}



bool LocationResult::_world2Grid(float x, float y, int* grid_x, int* grid_y) {
  pthread_mutex_lock(&picture_data_mutex_);
  *grid_x = ceil((x - map_info_.origen_x) / map_info_.resolution);
  *grid_y = ceil((y - map_info_.origen_y) / map_info_.resolution);
  pthread_mutex_unlock(&picture_data_mutex_);

  return _isInPicture(*grid_x, *grid_y);
}

bool LocationResult::_isInPicture(int grid_x, int grid_y) {
  pthread_mutex_lock(&picture_data_mutex_);
  if (grid_x < 0 || grid_x >= map_info_.width ||
      grid_y < 0 || grid_y >= map_info_.height) {
    pthread_mutex_unlock(&picture_data_mutex_);
    return false;
  }
  pthread_mutex_unlock(&picture_data_mutex_);
  return true;
}



void LocationResult::_ladarInMap(
  const RadarSensoryMessage& ladar_info, cv::Mat* map) {
  int current_grid_x, current_grid_y;
  float mfXPos = ladar_info.mstruRadarMessage.mstruRadarHeaderData.mfXPos;
  float mfYPos = ladar_info.mstruRadarMessage.mstruRadarHeaderData.mfYPos;
  float mfTheta = ladar_info.mstruRadarMessage.mstruRadarHeaderData.mfTheta;
  if (!_world2Grid(mfXPos, mfYPos, &current_grid_x, &current_grid_y)) return;

  Coordinate::Transform transform;
  transform.SetPose1InGlobal(Eigen::Vector3d(mfXPos, mfYPos, mfTheta));
  for (auto iter : ladar_info.mstruRadarMessage.mstruSingleLayerData.mvPoints) {
    // 将雷达点转换至全局坐标下
    transform.SetPose2InPose1(Eigen::Vector3d(iter(0), iter(1), 0.0));
    Eigen::Vector3d in_global;
    transform.GetPose2InGlobal(&in_global);
    int grid_x, grid_y;
    if (!_world2Grid(in_global(0), in_global(1), &grid_x, &grid_y)) continue;
    cv::circle(*map, cv::Point2f(grid_x, grid_y), 2,
                  cv::Scalar(0, 0, 255), -1);
    // // 画线 灰色
    // LineMath::BresenhamLine line;
    // line.SetStartAndEndGrid(grid_x, grid_y, current_grid_x, current_grid_y);
    // int next_x, next_y;
    // while (line.GetNextGrid(&next_x, &next_y)) {
    //   if (!_isInPicture(next_x, next_y)) continue;
    //   // 显示为灰色
    //   (*map).at<cv::Vec3b>(next_y, next_x)[0] = 192;
    //   (*map).at<cv::Vec3b>(next_y, next_x)[1] = 192;
    //   (*map).at<cv::Vec3b>(next_y, next_x)[2] = 192;
    // }
  }
  // 显示粒子分布情况
  int sample_size = ladar_info.mstruRadarMessage.mstruMultiLayerData.size();
  cv::putText(*map, std::to_string(sample_size), cv::Point(10, 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);
  for (auto iter : ladar_info.mstruRadarMessage.mstruMultiLayerData) {
    int grid_x, grid_y;
    if (!_world2Grid(iter.x, iter.y, &grid_x, &grid_y)) continue;
    // 显示为绿色
    (*map).at<cv::Vec3b>(grid_y, grid_x)[0] = 255;
    (*map).at<cv::Vec3b>(grid_y, grid_x)[1] = 0;
    (*map).at<cv::Vec3b>(grid_y, grid_x)[2] = 0;
    // 显示粒子的方向
    // float end_pos_x = iter.x + 0.02 * cos(iter.z);
    // float end_pos_y = iter.y + 0.02 * sin(iter.z);
    // int end_grid_x, end_grid_y;
    // if (!_world2Grid(end_pos_x, end_pos_y, &end_grid_x, &end_grid_y))
    //    continue;
    // cv::arrowedLine(*map, cv::Point(current_grid_x, current_grid_y),
    //                 cv::Point(end_grid_x, end_grid_y),
    //                 cv::Scalar(0, 0, 0), 1, 8, 0, 0.01);
  }


  // 显示当前位姿朝向
  float end_pos_x = mfXPos + 0.3 * cos(mfTheta);
  float end_pos_y = mfYPos + 0.3 * sin(mfTheta);
  int end_grid_x, end_grid_y;
  if (!_world2Grid(end_pos_x, end_pos_y, &end_grid_x, &end_grid_y)) {
    SLAM_ERROR("show pos theta is error......");
  } else {
    cv::arrowedLine(*map, cv::Point(current_grid_x, current_grid_y),
                    cv::Point(end_grid_x, end_grid_y),
                    cv::Scalar(0, 255, 0), 1, 8, 0, 0.1);
  }
  return;
}



void LocationResult::_locationPosEvent(
  void *object, char *buf, const int size) {
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(object);
  gomros::common::Message<TotalOdomMessgae> location_pose;
  location_pose.FromCharArray(buf, size);
  OdometerMessage position = location_pose.pack.odom_oose;
  int grid_x, grid_y;
  if (!lp_this->_world2Grid(
      position.mclDeltaPosition.mfX, position.mclDeltaPosition.mfY,
      &grid_x, &grid_y)) {
    // SLAM_ERROR("location pos is error......");
    return;
  }
  pthread_mutex_lock(&lp_this->picture_data_mutex_);
  lp_this->is_get_pos_ = true;
  lp_this->show_mat_.at<cv::Vec3b>(grid_y, grid_x)[0] = 0;
  lp_this->show_mat_.at<cv::Vec3b>(grid_y, grid_x)[1] = 0;
  lp_this->show_mat_.at<cv::Vec3b>(grid_y, grid_x)[2] = 255;
  pthread_mutex_unlock(&lp_this->picture_data_mutex_);
  return;
}

void LocationResult::_gridMapEvent(void *object, char *buf, const int size) {
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(object);
  SimpleGridMap simple_grid_map;
  simple_grid_map.from_json_char_array(buf, size);
  if (simple_grid_map.map_info.miMapWidth <= 0 ||
      simple_grid_map.map_info.miMapHeight <= 0) {
    SLAM_WARN("get grid map width %f, height %f",
                simple_grid_map.map_info.miMapWidth,
                simple_grid_map.map_info.miMapHeight);
    return;
  }
  lp_this->grid_map_ = simple_grid_map;
  lp_this->is_get_map_ = true;
}

void *LocationResult::_displayMapPthread(void *param) {
  pthread_detach(pthread_self());
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(param);
  while (1) {
    if (lp_this->is_get_map_) {
      lp_this->is_get_map_ = false;
      lp_this->_showGridMap();
    } else {
      usleep(100000);
      continue;
    }
    pthread_mutex_lock(&lp_this->picture_data_mutex_);
    cv::Mat grid_map_mat = lp_this->show_mat_.clone();
    if (grid_map_mat.rows <= 0 || grid_map_mat.cols <= 0) {
      pthread_mutex_unlock(&lp_this->picture_data_mutex_);
      continue;
    }
    cv::namedWindow("grid_map", 0);
    cv::resizeWindow("grid_map",
      lp_this->map_info_.width, lp_this->map_info_.height);
    cv::imshow("grid_map", grid_map_mat);
    cv::waitKey(10);
    pthread_mutex_unlock(&lp_this->picture_data_mutex_);
    usleep(100000);
  }
  pthread_exit(NULL);
}




void LocationResult::_rawLadarEvent(void *object, char *buf, const int size) {
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(object);
  RadarSensoryMessage radar_data;
  radar_data.from_char_array(buf, size);
  pthread_mutex_lock(&lp_this->raw_ladar_data_mutex_);
  lp_this->raw_ladar_datas_.push_back(radar_data);
  pthread_mutex_unlock(&lp_this->raw_ladar_data_mutex_);
  return;
}

void *LocationResult::_displayLadarInMapPthread(void *param) {
  pthread_detach(pthread_self());
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(param);
  while (1) {
    RadarSensoryMessage ladar_info;
    pthread_mutex_lock(&lp_this->raw_ladar_data_mutex_);
    if (lp_this->raw_ladar_datas_.empty()) {
      usleep(1000);
      pthread_mutex_unlock(&lp_this->raw_ladar_data_mutex_);
      continue;
    }
    ladar_info = lp_this->raw_ladar_datas_.front();
    lp_this->raw_ladar_datas_.pop_front();
    pthread_mutex_unlock(&lp_this->raw_ladar_data_mutex_);
    if (ladar_info.mstruRadarMessage.mstruRadarHeaderData.mcPosValid != 1) {
      usleep(1000);
      continue;
    }

    pthread_mutex_lock(&lp_this->picture_data_mutex_);
    cv::Mat show_mat = lp_this->show_mat_.clone();
    cv::namedWindow("ladar_in_map", 0);
    cv::resizeWindow("ladar_in_map",
      lp_this->map_info_.width, lp_this->map_info_.height);
    lp_this->_ladarInMap(ladar_info, &show_mat);

    cv::imshow("ladar_in_map", show_mat);
    pthread_mutex_unlock(&lp_this->picture_data_mutex_);
    cv::waitKey(20);
  }
  pthread_exit(NULL);
}

void *LocationResult::_displayReflectorMappingPthread(void *param) {
  pthread_detach(pthread_self());
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(param);
  int count = 0;
  while (1) {
    pthread_mutex_lock(&lp_this->reflector_mapping_mutex_);
    if (lp_this->reflector_mapping_datas_.empty()) {
      usleep(100);
      pthread_mutex_unlock(&lp_this->reflector_mapping_mutex_);
      continue;
    }
    RadarSensoryMessage radar_data = lp_this->reflector_mapping_datas_.front();
    lp_this->reflector_mapping_datas_.pop_front();
    pthread_mutex_unlock(&lp_this->reflector_mapping_mutex_);
    lp_this->_changeMapInfo(
      radar_data.mstruRadarMessage.mstruRadarHeaderData.mfXPos,
      radar_data.mstruRadarMessage.mstruRadarHeaderData.mfYPos);
    pthread_mutex_lock(&lp_this->picture_data_mutex_);
    cv::Mat show_mat = lp_this->show_mat_.clone();
    cv::namedWindow("reflector_mapping", 0);
    cv::resizeWindow("reflector_mapping",
      lp_this->map_info_.width, lp_this->map_info_.height);
    pthread_mutex_unlock(&lp_this->picture_data_mutex_);
    if (!lp_this->_reflectorMapping(radar_data, &show_mat))
      continue;

    cv::imshow("reflector_mapping", show_mat);
    // char picture_buff[512];
    // snprintf(picture_buff, sizeof(picture_buff), \
    //         "./out/submap_%02d.png", count++);
    // cv::imwrite(picture_buff, show_mat);
    cv::waitKey(2);
  }

  pthread_exit(NULL);
}

void LocationResult::_ReflectorMappingEvent(
  void *object, char *buf, const int size) {
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(object);
  RadarSensoryMessage radar_data;
  radar_data.from_char_array(buf, size);
  pthread_mutex_lock(&lp_this->reflector_mapping_mutex_);
  if (radar_data.mstruRadarMessage.mstruRadarHeaderData.mcPosValid) {
    lp_this->reflector_mapping_datas_.push_back(radar_data);
  }
  pthread_mutex_unlock(&lp_this->reflector_mapping_mutex_);
}


bool LocationResult::_reflectorMapping(
  const RadarSensoryMessage& ladar_info, cv::Mat* map) {
  static int reflector_count = 0;
  int current_grid_x, current_grid_y;
  float mfXPos = ladar_info.mstruRadarMessage.mstruRadarHeaderData.mfXPos;
  float mfYPos = ladar_info.mstruRadarMessage.mstruRadarHeaderData.mfYPos;
  float mfTheta = ladar_info.mstruRadarMessage.mstruRadarHeaderData.mfTheta;
  if (!_world2Grid(mfXPos, mfYPos, &current_grid_x, &current_grid_y))
    return false;

  cv::putText(*map, std::to_string(reflector_count++), cv::Point(10, 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);
  // 显示全部雷达信息  -- 雷达坐标系下  -- 转至全局坐标系
  Coordinate::Transform transform;
  transform.SetPose1InGlobal(Eigen::Vector3d(mfXPos, mfYPos, mfTheta));
  transform.SetPose2InPose1(Eigen::Vector3d(
    config_.mapping_config.sensor_mount.radar_position_x,
    config_.mapping_config.sensor_mount.radar_position_y,
    config_.mapping_config.sensor_mount.radar_position_theta));
  Eigen::Vector3d ladar_in_global;
  transform.GetPose2InGlobal(&ladar_in_global);
  transform.SetPose1InGlobal(ladar_in_global);
  for (auto iter : ladar_info.mstruRadarMessage.mstruSingleLayerData.mvPoints) {
      transform.SetPose2InPose1(Eigen::Vector3d(iter(0), iter(1), 0.0));
      Eigen::Vector3d in_global;
      transform.GetPose2InGlobal(&in_global);
      int grid_x, grid_y;
      if (!_world2Grid(in_global(0), in_global(1), &grid_x, &grid_y)) continue;
      cv::circle(*map, cv::Point2f(grid_x, grid_y), 2,
                  cv::Scalar(192, 192, 192), -1);
      // LineMath::BresenhamLine line;
      // line.SetStartAndEndGrid(grid_x, grid_y,
      //  current_grid_x, current_grid_y);
      // int next_x, next_y;
      // while (line.GetNextGrid(&next_x, &next_y)) {
      //   if (!_isInPicture(next_x, next_y)) continue;
      //   // 显示为灰色
      //   (*map).at<cv::Vec3b>(next_y, next_x)[0] = 192;
      //   (*map).at<cv::Vec3b>(next_y, next_x)[1] = 192;
      //   (*map).at<cv::Vec3b>(next_y, next_x)[2] = 192;
      // }
  }

  // 转至全局坐标系下面
  transform.SetPose1InGlobal(Eigen::Vector3d(mfXPos, mfYPos, mfTheta));
  // 显示目前建图状态
  for (auto iter : ladar_info.mstruRadarMessage.mstruMultiLayerData) {
    // 将雷达点转换至全局坐标下  -- 地图红色
    int grid_x, grid_y;
    DisPlayResult::ReflectorType type =
      static_cast<DisPlayResult::ReflectorType>(iter.z);
    if (DisPlayResult::ReflectorType::InState == type) {
      if (!_world2Grid(iter.x, iter.y, &grid_x, &grid_y)) continue;
      cv::circle(*map, cv::Point2f(grid_x, grid_y), 5,
                  DisPlayResult::GetScalarByType(type), -1);
    } else if (DisPlayResult::ReflectorType::Identify == type) {
      transform.SetPose2InPose1(Eigen::Vector3d(iter.x, iter.y, 0.0));
      Eigen::Vector3d in_global;
      transform.GetPose2InGlobal(&in_global);
      if (!_world2Grid(in_global(0), in_global(1), &grid_x, &grid_y)) continue;
      // 新识别的绿色
      cv::circle(*map, cv::Point2f(grid_x, grid_y), 5,
                  DisPlayResult::GetScalarByType(type), -1);
      LineMath::BresenhamLine line;
      line.SetStartAndEndGrid(grid_x, grid_y, current_grid_x, current_grid_y);
      int next_x, next_y;
      while (line.GetNextGrid(&next_x, &next_y)) {
        if (!_isInPicture(next_x, next_y)) continue;
        // 显示为灰色
        (*map).at<cv::Vec3b>(next_y, next_x)[0] = 0;
        (*map).at<cv::Vec3b>(next_y, next_x)[1] = 255;
        (*map).at<cv::Vec3b>(next_y, next_x)[2] = 0;
      }
    }
  }
  // 显示当前位姿朝向
  float end_pos_x = mfXPos + 0.3 * cos(mfTheta);
  float end_pos_y = mfYPos + 0.3 * sin(mfTheta);
  int end_grid_x, end_grid_y;
  if (!_world2Grid(end_pos_x, end_pos_y, &end_grid_x, &end_grid_y)) {
    SLAM_ERROR("show pos theta is error......");
  } else {
    cv::arrowedLine(*map, cv::Point(current_grid_x, current_grid_y),
                    cv::Point(end_grid_x, end_grid_y),
                    cv::Scalar(0, 255, 0), 1, 8, 0, 0.1);
  }
  return true;
}



void LocationResult::_showGridMap() {
  cv::Mat grid_map_mat =
    cv::Mat(cv::Size(grid_map_.map_info.miMapWidth,
                      grid_map_.map_info.miMapHeight),
    CV_8UC3, cv::Scalar(192, 192, 192));
  for (int y = 0; y < grid_map_.map_info.miMapHeight; y++) {
    for (int x = 0; x < grid_map_.map_info.miMapWidth; x++) {
    double mx = grid_map_.map_info.MapXToRealX(x);
    double my = grid_map_.map_info.MapYToRealY(y);
      char val = grid_map_.get(mx, my);
      switch (val) {
        case 100 :
          grid_map_mat.at<cv::Vec3b>(y, x)[0] = 0;
          grid_map_mat.at<cv::Vec3b>(y, x)[1] = 0;
          grid_map_mat.at<cv::Vec3b>(y, x)[2] = 0;
        break;
        case 0 :
          grid_map_mat.at<cv::Vec3b>(y, x)[0] = 255;
          grid_map_mat.at<cv::Vec3b>(y, x)[1] = 255;
          grid_map_mat.at<cv::Vec3b>(y, x)[2] = 255;
        break;
        default :
          grid_map_mat.at<cv::Vec3b>(y, x)[0] = 192;
          grid_map_mat.at<cv::Vec3b>(y, x)[1] = 192;
          grid_map_mat.at<cv::Vec3b>(y, x)[2] = 192;
        break;
      }
    }
  }

  pthread_mutex_lock(&picture_data_mutex_);
  map_info_.resolution =  grid_map_.map_info.mdResolution;
  map_info_.width = grid_map_.map_info.miMapWidth;
  map_info_.height = grid_map_.map_info.miMapHeight;
  map_info_.origen_x = grid_map_.map_info.mdOriginXInWorld;
  map_info_.origen_y = grid_map_.map_info.mdOriginYInWorld;
  SLAM_INFO("get new grid map (%f %f) width %d height %d",
            map_info_.origen_x, map_info_.origen_y,
            map_info_.width, map_info_.height);
  show_mat_ = grid_map_mat.clone();
  cv::namedWindow("grid_map", 0);
  cv::resizeWindow("grid_map", map_info_.width, map_info_.height);
  cv::imshow("grid_map", grid_map_mat);
  cv::imwrite("new_map.png", grid_map_mat);
  cv::waitKey(10);
  pthread_mutex_unlock(&picture_data_mutex_);
  return;
}

void LocationResult::_displayConfidenceEllipse(const Eigen::Vector2d& mean,
  const Eigen::Matrix2d& cov, cv::Mat& show_mat) {
  Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
  // 特征值
  Eigen::Matrix2d value = es.pseudoEigenvalueMatrix();
  // 特征向量
  Eigen::Matrix2d vector = es.pseudoEigenvectors();

  float theta = value(0, 0) > value(1, 1) ?
              std::atan2(vector(1, 0), vector(0, 0)) :
              std::atan2(vector(1, 1), vector(0, 1));
  theta *= -180.0 / M_PI;
  float resolution = config_.mapping_config.mapping_resolution;
  int grid_x, grid_y;
  if (!_world2Grid(mean(0), mean(1), &grid_x, &grid_y)) return;

  cv::ellipse(show_mat, cv::Point(grid_x, grid_y),
      cv::Size(std::sqrt(value(0, 0)) * 10 / resolution,
      std::sqrt(value(1, 1)) * 10 / resolution),
      theta, 0, 360, cv::Scalar(0, 0, 255));
}


void *LocationResult::_displayReflectorLocationPthread(void *param) {
  pthread_detach(pthread_self());
  SLAM_INFO("begin reflection location pthread~~~~~~~");
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(param);
  int count = 0;
  while (1) {
    pthread_mutex_lock(&lp_this->reflector_location_mutex_);
    if (lp_this->reflector_location_datas_.empty()) {
      usleep(100);
      pthread_mutex_unlock(&lp_this->reflector_location_mutex_);
      continue;
    }
    RadarSensoryMessage radar_data = lp_this->reflector_location_datas_.front();
    lp_this->reflector_location_datas_.pop_front();
    pthread_mutex_unlock(&lp_this->reflector_location_mutex_);
    pthread_mutex_lock(&lp_this->picture_data_mutex_);
    cv::Mat show_mat = lp_this->show_mat_.clone();
    cv::namedWindow("reflector_loation", 0);
    cv::resizeWindow("reflector_loation",
      lp_this->map_info_.width, lp_this->map_info_.height);
    pthread_mutex_unlock(&lp_this->picture_data_mutex_);
    if (!lp_this->_reflectorLocation(radar_data, &show_mat))
      continue;

    cv::imshow("reflector_loation", show_mat);
    // char picture_buff[512];
    // snprintf(picture_buff, sizeof(picture_buff), \
            //         "./out/reflector_location_%02d.png", count++);
    // cv::imwrite(picture_buff, show_mat);
    cv::waitKey(2);
  }

  pthread_exit(NULL);
}

void LocationResult::_ReflectorLocationEvent(
  void *object, char *buf, const int size) {
  LocationResult *lp_this = reinterpret_cast<LocationResult *>(object);
  RadarSensoryMessage radar_data;
  radar_data.from_char_array(buf, size);
  pthread_mutex_lock(&lp_this->reflector_location_mutex_);
  if (radar_data.mstruRadarMessage.mstruRadarHeaderData.mcPosValid) {
    lp_this->reflector_location_datas_.push_back(radar_data);
  }
  pthread_mutex_unlock(&lp_this->reflector_location_mutex_);
}

bool LocationResult::_reflectorLocation(
  const RadarSensoryMessage& ladar_info, cv::Mat* map) {
  static int reflector_count = 0;
  int current_grid_x, current_grid_y;
  float mfXPos = ladar_info.mstruRadarMessage.mstruRadarHeaderData.mfXPos;
  float mfYPos = ladar_info.mstruRadarMessage.mstruRadarHeaderData.mfYPos;
  float mfTheta = ladar_info.mstruRadarMessage.mstruRadarHeaderData.mfTheta;
  if (!_world2Grid(mfXPos, mfYPos, &current_grid_x, &current_grid_y))
    return false;

  cv::putText(*map, std::to_string(reflector_count++), cv::Point(10, 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);
  // 显示全部雷达信息  -- 雷达坐标系下  -- 转至全局坐标系
  Coordinate::Transform transform;
  transform.SetPose1InGlobal(Eigen::Vector3d(mfXPos, mfYPos, mfTheta));
  transform.SetPose2InPose1(Eigen::Vector3d(
    config_.mapping_config.sensor_mount.radar_position_x,
    config_.mapping_config.sensor_mount.radar_position_y,
    config_.mapping_config.sensor_mount.radar_position_theta));
  Eigen::Vector3d ladar_in_global;
  transform.GetPose2InGlobal(&ladar_in_global);
  transform.SetPose1InGlobal(ladar_in_global);
  for (auto iter : ladar_info.mstruRadarMessage.mstruSingleLayerData.mvPoints) {
      transform.SetPose2InPose1(Eigen::Vector3d(iter(0), iter(1), 0.0));
      Eigen::Vector3d in_global;
      transform.GetPose2InGlobal(&in_global);
      int grid_x, grid_y;
      if (!_world2Grid(in_global(0), in_global(1), &grid_x, &grid_y)) continue;
      cv::circle(*map, cv::Point2f(grid_x, grid_y), 2,
                  cv::Scalar(192, 192, 192), -1);
  }

  // 显示目前建图状态
  for (auto iter : ladar_info.mstruRadarMessage.mstruMultiLayerData) {
    // 将雷达点转换至全局坐标下  -- 地图红色
    int grid_x, grid_y;
    DisPlayResult::ReflectorType type =
      static_cast<DisPlayResult::ReflectorType>(iter.z);
    if (!_world2Grid(iter.x, iter.y, &grid_x, &grid_y)) continue;
    cv::circle(*map, cv::Point2f(grid_x, grid_y), 5,
                DisPlayResult::GetScalarByType(type), -1);
    if (DisPlayResult::ReflectorType::Identify == type) {
      LineMath::BresenhamLine line;
      line.SetStartAndEndGrid(grid_x, grid_y, current_grid_x, current_grid_y);
      int next_x, next_y;
      while (line.GetNextGrid(&next_x, &next_y)) {
        if (!_isInPicture(next_x, next_y)) continue;
        // 显示为灰色
        (*map).at<cv::Vec3b>(next_y, next_x)[0] = 0;
        (*map).at<cv::Vec3b>(next_y, next_x)[1] = 255;
        (*map).at<cv::Vec3b>(next_y, next_x)[2] = 0;
      }
    }
  }
  // 显示当前位姿朝向
  float end_pos_x = mfXPos + 0.3 * cos(mfTheta);
  float end_pos_y = mfYPos + 0.3 * sin(mfTheta);
  int end_grid_x, end_grid_y;
  if (!_world2Grid(end_pos_x, end_pos_y, &end_grid_x, &end_grid_y)) {
    SLAM_ERROR("show pos theta is error......");
  } else {
    cv::arrowedLine(*map, cv::Point(current_grid_x, current_grid_y),
                    cv::Point(end_grid_x, end_grid_y),
                    cv::Scalar(0, 255, 0), 1, 8, 0, 0.1);
  }
  return true;
}



}  // namespace Display
