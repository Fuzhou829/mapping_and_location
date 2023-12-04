/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-09-27 15:02:35
 * @LastEditTime: 2023-10-26 10:51:13
 * @Author: lcfc-desktop
 */
#pragma once

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "Eigen/Dense"

#include "common_lib/gomros.h"
#include "message_lib/cmd_message.h"
#include "message_lib/device_state.h"
#include "message_lib/odometer_message.h"
#include "message_lib/pose_and_odom_message.h"
#include "message_lib/position_message.h"
#include "message_lib/radar_message.h"
#include "message_lib/simple_grid_map.h"
#include "message_lib/iosystem_message.h"
#include "message_lib/speed_message.h"

#include "location/location_manager_impl.h"
#include "location/location_area.h"
#include "include/config_struct.h"
#include "record_data/record_data.h"

#include "mapping/mapping_interface.h"
#include "fusion_mapping/fusion_mapping.h"
#include "karto_mapping/karto_mapping.h"
#include "reflector_mapping/reflector_mapping.h"
#include "reflector_mapping/trilateral_mapping.h"
#include "optimize_mapping/optimize_mapping.h"

#include "fusion_mapping/fusion_map_info.h"
#include "common/tool.h"
#include "common/logger.h"

namespace gomros {
namespace data_process {
namespace mapping_and_location {

class MappingAndLocationImpl {
 public:
  using RawPublisher = gomros::common::RawPublisher;
  using CallBackEvent = gomros::common::CallBackEvent;
  using SimpleGridMap = gomros::message::SimpleGridMap;
  using MapInfo = gomros::message::MapInfo;
  using Position = gomros::message::Position;
  using Node = gomros::common::Node;
  using OdometerMessage = gomros::message::OdometerMessage;
  using DeviceState = gomros::message::DeviceState;
  using RadarSensoryMessage = gomros::message::RadarSensoryMessage;
  using DMreaderSensoryMessage = gomros::message::DMreaderSensoryMessage;
  using ImuSensoryMessage = gomros::message::ImuSensoryMessage;
  using CustomTaskString = gomros::message::CustomTaskString;

 public:
  // 建图及定位构造函数
  explicit MappingAndLocationImpl(const MappingAndLocationConfig& config);

  ~MappingAndLocationImpl();

  /**
   * @describes: 利用充电点进行重定位
   * @return {*}
   */
  void ChargeRelocation();

 private:
  /* 加载地图及定位等初始化信息 */
  void _loadPoseFromFile();
  // 从文件中加载地图信息
  void _loadMapNameFromFile(const std::string& file_name);
  /**
   * @describes: 从指定地图文件中读取地图数据
   * @param map_file_name 地图名
   * @param is_need_charge 是否需要充电点信息
   * @return {*}
   */  
  bool _loadMapDataFromFile(std::string map_file_name,
                            bool is_need_charge = false);
  void _loadGridMap(const Json::Value& map_json);
  void _loadLandMarkMap(const Json::Value& map_json);
  void _loadQrMap(const Json::Value& map_json);
  void _loadLoactionAreaInfo(const Json::Value& map_json);
  bool _loadChargePos(const Json::Value& map_json);

  void _checkLadarMessage(const RadarSensoryMessage& ladar_message);

  // 开始建图模块
  void _startMapping();
  // 建图线程
  static void* _stopMappingThread(void* ptr);
  // 地图合并线程
  static void* _mergeMappingThread(void* ptr);

  // 发布机器目前的状态
  void _publishRobotState();

  /* 消息订阅回调 */
  // 机器定位状态信息发布
  static void* _publishRobotStateThread(void* ptr);
  // 接受机器当前速度
  static void _robotValCallBackFunc(void *object, char *buf, const int size);
  // 接收急停等指令
  static void _emergeCallBackFunc(void *object, char *buf, const int size);
  // 接收odom数据信息
  static void _odomCallBackFunc(void* object, char* buf, const int size);
  // 接收雷达数据
  static void _radarCallBackFunc(void* object, char* buf, const int size);
  // 接收imu数据
  static void _imuCallBackFunc(void* object, char* buf, const int size);
  // 接收qr数据
  static void _qrCallBackFunc(void* object, char* buf, const int size);
  // 接收开始定位消息
  static void _startLocateCallBack(void* object, char* buf, const int size);
  // 接收开始建图消息
  static void _startMappingCallBack(void* object, char* buf, const int size);
  // 接收终止建图消息
  static void _stopMappingCallBack(void* object, char* buf, const int size);
  // 接受标定相关指令
  static void _calibrationQrCameraCallBack(
    void* object, char* buf, const int size);
  // 接收切换楼层指令
  static void _upDateMapCallBack(void* object, char* buf, const int size);

  // 保留最新位姿给服务器
  void _savePoseToServer(const Position& current_pose);


 private:
  Node* node_ = nullptr;
  RawPublisher* state_publisher_;
  RawPublisher* pose_publisher_;
  RawPublisher* qr_pos_publisher_;
  Position initial_position_;
  Position radar_pose_base_link_;

  pthread_mutex_t last_odom_mutex_;
  OdometerMessage last_odom_message_;

  std::shared_ptr<DeviceState> device_state_;

  SimpleGridMap* map_shared_pointer_;  // 指向栅格地图的指针
  std::map<int, QRCoordinate> qr_map_;
  Json::Value merge_map_value_;   // 用于合并地图的信息
  std::shared_ptr<MappingInterface> mapping_module_;  // 建图模块

  // TODO(r) 后续需要进行调整代码结构
  std::shared_ptr<LocationManagerImpl> location_module_;  // 定位模块
  std::map<LocationType, std::vector<LocationArea>> location_areas_;


  pthread_t send_pose_thread_;
  pthread_t mapping_thread_;
  pthread_t merge_map_thread_;

  // for test
  pthread_t qr_thread_;

  pthread_mutex_t robot_vel_mutex_;
  float robot_line_vel_;
  float robot_angle_vel_;
  bool is_emerge_;
  bool is_move_;
  bool is_calibration_camera_;
  bool is_stop_mapping_;
  int robot_stop_count_;
  double total_odom_;
  std::string save_map_name_;
  string map_file_full_path_;
  MappingAndLocationConfig mapping_and_location_config_;
  // 收集定位测试数据
  std::shared_ptr<Record::SensorData> record_sensor_data_;
};

}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
