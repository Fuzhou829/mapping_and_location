/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Date: 2022-09-28 09:42:40
 * @LastEditTime: 2023-11-29 19:47:15
 * @Author: lcfc-desktop
 */
#include "include/mapping_and_location_impl.h"


#include "joint_map/joint_map_interface.h"
#include "mapping_and_location/MappingAndLocation.h.in"

namespace gomros {
namespace data_process {
namespace mapping_and_location {
MappingAndLocationImpl::MappingAndLocationImpl(
    const MappingAndLocationConfig &config) {
  pthread_mutex_init(&last_odom_mutex_, nullptr);
  pthread_mutex_init(&robot_vel_mutex_, nullptr);
  // 配置日志打印
  gomros::common::SLAMLogger::SetLoggerConfig(
      (gomros::common::LOG_LEVEL)config.log_level, config.log_file_name,
      config.is_printf_to_terminal);
  // add version info
  SLAM_INFO("version = %s", version.c_str());
  SLAM_INFO("compile_time = %s", compile_time.c_str());

  mapping_and_location_config_ = config;
  radar_pose_base_link_.mfX =
    mapping_and_location_config_.mapping_config.sensor_mount.radar_position_x;
  radar_pose_base_link_.mfY =
    mapping_and_location_config_.mapping_config.sensor_mount.radar_position_y;
  radar_pose_base_link_.mfTheta = mapping_and_location_config_.mapping_config.
                                  sensor_mount.radar_position_theta;
  SLAM_INFO("radar pos base link (%f %f %f)",
            radar_pose_base_link_.mfX, radar_pose_base_link_.mfY,
            radar_pose_base_link_.mfTheta);
  is_stop_mapping_ = true;
  is_emerge_ = true;
  is_move_ = false;
  is_calibration_camera_ = false;
  map_shared_pointer_ = nullptr;
  robot_line_vel_ = 1.01;
  robot_angle_vel_ = 0.41;

  std::string file_name =
      mapping_and_location_config_.mapping_config.map_config_file_path +
      "/" + "map_config.json";
  _loadMapNameFromFile(file_name);

  // XXX(r) 机器状态应该从外部进行订阅
  device_state_ = std::make_shared<DeviceState>(DeviceState());
  device_state_->mcChassisState = SYSTEM_STATE_INIT_POSE;

  record_sensor_data_ = std::make_shared<Record::SensorData>(
    config.mapping_config.record_config, "./location_data/");
  record_sensor_data_->StartRecord();
  SLAM_INFO("need record data %d",
             config.mapping_config.record_config.is_need_record_data);
  // 选择建图模式
  if (MappingType::Fusion == mapping_and_location_config_.mapping_type) {
    SLAM_INFO("now choose fusion mapping");
    mapping_module_ =
        std::make_shared<FusionMapping>(config.mapping_config);
  } else if (MappingType::Karto == mapping_and_location_config_.mapping_type) {
    SLAM_INFO("now choose KartoMapping mapping");
    mapping_module_ =
        std::make_shared<KartoMapping>(config.mapping_config);
  } else if (MappingType::Reflector ==
            mapping_and_location_config_.mapping_type) {
    SLAM_INFO("now choose Reflector mapping");
    mapping_module_ =
        std::make_shared<ReflectorMapping>(config.mapping_config);
  } else if (MappingType::Trilateral_Mapping ==
            mapping_and_location_config_.mapping_type) {
    SLAM_INFO("now choose TrilateralMapping");
    mapping_module_ =
        std::make_shared<TrilateralMapping>(config.mapping_config);
  } else if (MappingType::Optimize_Mapping ==
            mapping_and_location_config_.mapping_type) {
    SLAM_INFO("now choose Optimize_Mapping");
    mapping_module_ =
        std::make_shared<OptimizeMapping>(config.mapping_config);
  } else {
    SLAM_WARN("no mapping type now choose fusion mapping");
    mapping_module_ =
        std::make_shared<FusionMapping>(config.mapping_config);
  }

  location_module_ = std::make_shared<LocationManagerImpl>(
      config.location_config, device_state_);
  location_module_->SetDeviceState(device_state_);

  SLAM_INFO("sub_odom_topic = %s, sub_radar_topic= %s",
            config.sub_odom_topic.c_str(), config.sub_radar_topic.c_str());
  SLAM_INFO("sub_imu_topic = %s, sub_qr_topic= %s",
            config.sub_imu_topic.c_str(), config.sub_qr_topic.c_str());

  node_ = new Node(config.node_name);

  state_publisher_ = node_->AdvertiseTopic(config.ad_state_topic);
  pose_publisher_ = node_->AdvertiseTopic(config.ad_pose_topic);
  qr_pos_publisher_ = node_->AdvertiseTopic("qr_location_protect");

  // 订阅车体速度
  CallBackEvent robot_val_event{this, _robotValCallBackFunc};
  node_->SubscribeTopic("speed_topic", robot_val_event);
  // 增加急停判断，用于处理imu数据
  CallBackEvent emerge_event{this, _emergeCallBackFunc};
  node_->SubscribeTopic("io_subsystem/sensory", emerge_event);
  CallBackEvent odom_event{this, _odomCallBackFunc};
  node_->SubscribeTopic(config.sub_odom_topic, odom_event);
  CallBackEvent radar_event{this, _radarCallBackFunc};
  node_->SubscribeTopic(config.sub_radar_topic, radar_event);

  CallBackEvent imu_event{this, _imuCallBackFunc};
  node_->SubscribeTopic(config.sub_imu_topic, imu_event);
  CallBackEvent qr_event{this, _qrCallBackFunc};
  node_->SubscribeTopic(config.sub_qr_topic, qr_event);

  CallBackEvent start_location_event{this, _startLocateCallBack};
  node_->SubscribeTopic(config.sub_global_locate_cmd_topic,
                        start_location_event);
  CallBackEvent start_mapping_event{this, _startMappingCallBack};
  node_->SubscribeTopic(config.sub_start_mapping_cmd_topic,
                        start_mapping_event);
  CallBackEvent stop_mapping_event{this, _stopMappingCallBack};
  node_->SubscribeTopic(config.sub_stop_mapping_cmd_topic, stop_mapping_event);
  CallBackEvent calibration_event{this, _calibrationQrCameraCallBack};
  node_->SubscribeTopic(config.sub_calibration_cmd_topic, stop_mapping_event);

  CallBackEvent updata_map_event{this, _upDateMapCallBack};
  node_->SubscribeTopic("tcp_server_run_string", updata_map_event);

  node_->Debug();
  robot_stop_count_ = 128;
  if (_loadMapDataFromFile(map_file_full_path_)) {
    _loadPoseFromFile();
    location_module_->SetIniPose(initial_position_);
    location_module_->StartGlobalLocating();
    // SLAM_DEBUG("create robot state thread<<<<<\n");
    pthread_create(&send_pose_thread_, NULL, _publishRobotStateThread, this);
  } else {
    SLAM_ERROR("load map error!!!!!!!!!!!\n");
  }
  _publishRobotState();
}

MappingAndLocationImpl::~MappingAndLocationImpl() {
  pthread_join(send_pose_thread_, NULL);
  delete node_;
}

void MappingAndLocationImpl::ChargeRelocation() {
  device_state_->mcChassisState = SYSTEM_STATE_INIT_POSE;
  _publishRobotState();
  location_module_->SetHaveMap(false);
  location_module_->SetHavePose(false);
  SLAM_INFO("begin charge relocation, and choose map info is %s",
            map_file_full_path_.c_str());
  if (_loadMapDataFromFile(map_file_full_path_, true)) {
    location_module_->SetIniPose(initial_position_, true);
    location_module_->StartGlobalLocating(false);
  }
}

void MappingAndLocationImpl::_loadPoseFromFile() {
  std::string data_string;
  char filename[128], filename1[128];
  SLAM_INFO("init pos dir is >>> %s", mapping_and_location_config_.
                  location_config.init_pose_file_path.c_str());
  snprintf(filename, sizeof(filename), "%s/zero.json",
           mapping_and_location_config_.location_config.
           init_pose_file_path.c_str());
  snprintf(filename1, sizeof(filename1), "%s/zero1.json",
           mapping_and_location_config_.location_config.
           init_pose_file_path.c_str());
  if (!internal_common::Read(filename, &data_string) &&
      !internal_common::Read(filename1, &data_string)) {
    initial_position_.mfX = 0;
    initial_position_.mfY = 0;
    initial_position_.mfTheta = 0;
    initial_position_.mlTimestamp = gomros::common::GetCurrentTime_us();
    total_odom_ = 0;
    SLAM_ERROR("open init pos file error!!!!!!!\n");
    return;
  }
  Json::Reader reader;
  Json::Value data_json;
  initial_position_.mlTimestamp = gomros::common::GetCurrentTime_us();
  if (!reader.parse(data_string, data_json)) {
    initial_position_.mfX = 0;
    initial_position_.mfY = 0;
    initial_position_.mfTheta = 0;
    initial_position_.mlTimestamp = gomros::common::GetCurrentTime_us();
    total_odom_ = 0;
    SLAM_ERROR("parse map data failed.......\n");
    return;
  } else {
    SLAM_DEBUG("get init pos successful<<<<<<<<<\n");
    initial_position_.mfX = data_json["zero_pos"]["AgvX"].asFloat();
    initial_position_.mfY = data_json["zero_pos"]["AgvY"].asFloat();
    initial_position_.mfTheta = data_json["zero_pos"]["AgvTheta"].asFloat();
    total_odom_ = data_json["zero_pos"]["Total_odom"].asDouble();
    // TODO(r) 判断数据是否异常
    // if (initial_position_.mfX < x_min_ || initial_position_.mfX > x_max_ ||
    //     initial_position_.mfY < y_min_ || initial_position_.mfY > y_max_) {
    //    SLAM_ERROR("please check your init pos~~~~~~~~~~~~~~~");
    // }
    SLAM_INFO("init pos (%f %f %f) total_odom_ %f",
             initial_position_.mfX, initial_position_.mfY,
             initial_position_.mfTheta, total_odom_);
  }
}

void MappingAndLocationImpl::_loadMapNameFromFile(
    const std::string &file_name) {
  std::string data_string;
  std::string map_config_full_path =
      mapping_and_location_config_.mapping_config.map_config_file_path +
      "/" + "map_config.json";
  if (internal_common::Read(map_config_full_path.c_str(), &data_string)) {
    Json::Reader reader;
    Json::Value data_json;
    if (!reader.parse(data_string, data_json)) {
      SLAM_ERROR("parse map data error");
      map_file_full_path_ =
          mapping_and_location_config_.mapping_config.map_data_file_path +
          "/" + "ale.smap";
      return;
    } else {
      map_file_full_path_ =
          mapping_and_location_config_.mapping_config.map_data_file_path +
          "/" + data_json["defaultMap"].asString();
    }
  } else {
    SLAM_ERROR("load map file filed");
    map_file_full_path_ =
        mapping_and_location_config_.mapping_config.map_data_file_path +
        "/" + "ale.smap";
    return;
  }
}

bool MappingAndLocationImpl::_loadMapDataFromFile(std::string map_file_name,
  bool is_need_charge) {
  char filename[128];
  SLAM_DEBUG("load map dir is %s", map_file_name.c_str());
  std::string map_data;
  if (!internal_common::Read(map_file_name.c_str(), &map_data)) {
    SLAM_ERROR("read map file failed....");
    return false;
  }
  Json::Reader reader;
  Json::Value map_json;
  if (!reader.parse(map_data, map_json)) {
    SLAM_ERROR("parse map failed.....");
    return false;
  }
  // 1. 加载栅格地图
  _loadGridMap(map_json);
  // 2.加载反光柱地图
  _loadLandMarkMap(map_json);
  // 3.加载qr地图
  _loadQrMap(map_json);
  // 4. 加载定位区域信息
  _loadLoactionAreaInfo(map_json);
  if (!is_need_charge) return true;
  // 5. 加载充电点信息
  bool is_get_charge = _loadChargePos(map_json);
  return is_get_charge;
}

void MappingAndLocationImpl::_loadGridMap(const Json::Value &map_json) {
  MapInfo info;
  float resolution = map_json["header"]["resolution"].asDouble();
  float origen_x = map_json["header"]["minPos"]["x"].asDouble();
  float origen_y = map_json["header"]["minPos"]["y"].asDouble();
  info.mdOriginXInWorld = origen_x;
  info.mdOriginYInWorld = origen_y;
  info.mdResolution = resolution;
  int width = info.RealXToMapX(map_json["header"]["maxPos"]["x"].asDouble());
  int height = info.RealYToMapY(map_json["header"]["maxPos"]["y"].asDouble());
  info.miMapWidth = width;
  info.miMapHeight = height;
  if (map_shared_pointer_ != nullptr) {
    delete map_shared_pointer_;
  }
  map_shared_pointer_ = new SimpleGridMap(info);
  SLAM_DEBUG("###resolution=%f,width=%d,height=%d,origin_x=%f,origin_y=%f",
             resolution, map_shared_pointer_->map_info.miMapWidth,
             map_shared_pointer_->map_info.miMapHeight,
             map_shared_pointer_->map_info.mdOriginXInWorld,
             map_shared_pointer_->map_info.mdOriginYInWorld);
  map_shared_pointer_->datas = std::vector<char>(width * height, 0);

  for (int j = 0; j < map_json["normalPosList"].size(); j++) {
    int x = map_shared_pointer_->map_info.RealXToMapX(
        map_json["normalPosList"][j]["x"].asDouble());
    int y = map_shared_pointer_->map_info.RealYToMapY(
        map_json["normalPosList"][j]["y"].asDouble());
    int index = map_shared_pointer_->map_info.MapIndexToArraryIndex(x, y);
    if (index < map_shared_pointer_->datas.size()) {
      map_shared_pointer_
          ->datas[map_shared_pointer_->map_info.MapIndexToArraryIndex(x, y)] =
          100;
    } else {
      SLAM_DEBUG("###resolution=%f,width=%d,height=%d,origin_x=%f,origin_y=%f",
                 resolution, map_shared_pointer_->map_info.miMapWidth,
                 map_shared_pointer_->map_info.miMapHeight,
                 map_shared_pointer_->map_info.mdOriginXInWorld,
                 map_shared_pointer_->map_info.mdOriginYInWorld);
    }
  }
  location_module_->SetMapData(map_shared_pointer_);
}

void MappingAndLocationImpl::_loadLandMarkMap(const Json::Value &map_json) {
  // 从json中读取
  std::map<int, std::map<int, Eigen::Vector2d>> map_info;
  internal_common::ReadLandmarkInfoFromJson(map_json, &map_info);

  location_module_->SetLandMarkMapData(map_info);
  SLAM_INFO("load map successful<<<<<<<<");
}

void MappingAndLocationImpl::_loadQrMap(const Json::Value &map_json) {
  qr_map_.clear();
  for (int i = 0; i < map_json["qr_pos_list"].size(); i++) {
    QRCoordinate qr_info;
    qr_info.tag_num = map_json["qr_pos_list"][i]["tag_num"].asInt();
    qr_info.mx = map_json["qr_pos_list"][i]["x"].asDouble();
    qr_info.my = map_json["qr_pos_list"][i]["y"].asDouble();
    qr_info.theta = map_json["qr_pos_list"][i]["theta"].asDouble();
    if (qr_map_.count(qr_info.tag_num)) continue;
    qr_map_.insert(std::make_pair(qr_info.tag_num, qr_info));
  }
  location_module_->SetQrMapData(qr_map_);
}

void MappingAndLocationImpl::_loadLoactionAreaInfo(
  const Json::Value &map_json) {
  location_areas_.clear();
  int area_count = map_json["advancedAreaList"].size();
  if (area_count <= 0) {
    SLAM_INFO("there is no appoint area~~~~~");
    return;
  }
  for (int i = 0; i < area_count; i++) {
    LocationArea area_info;
    std::string navigation_model =
        map_json["advancedAreaList"][i]["NavigateModel"].asString();
    if (navigation_model == "QRCode") {
      area_info.min_mx =
          map_json["advancedAreaList"][i]["LeftTopPoint"]["x"].asDouble();
      area_info.max_my =
          map_json["advancedAreaList"][i]["LeftTopPoint"]["y"].asDouble();
      area_info.max_mx =
          map_json["advancedAreaList"][i]["RightBottomPoint"]["x"].asDouble();
      area_info.min_my =
          map_json["advancedAreaList"][i]["RightBottomPoint"]["y"].asDouble();
      SLAM_INFO(">>>>qr_area is %f %f %f %f", area_info.min_mx,
                area_info.max_mx, area_info.min_my, area_info.max_my);
      if (location_areas_.count(LocationType::QR)) {
        location_areas_.at(LocationType::QR).push_back(area_info);
      } else {
        std::vector<LocationArea> qr_vector = {area_info};
        location_areas_.insert(std::make_pair(LocationType::QR, qr_vector));
      }
    }
    if (navigation_model == "Reflector") {
      area_info.min_mx =
          map_json["advancedAreaList"][i]["LeftTopPoint"]["x"].asDouble();
      area_info.max_my =
          map_json["advancedAreaList"][i]["LeftTopPoint"]["y"].asDouble();
      area_info.max_mx =
          map_json["advancedAreaList"][i]["RightBottomPoint"]["x"].asDouble();
      area_info.min_my =
          map_json["advancedAreaList"][i]["RightBottomPoint"]["y"].asDouble();
      SLAM_INFO(">>>>Reflector_area is %f %f %f %f", area_info.min_mx,
                area_info.max_mx, area_info.min_my, area_info.max_my);
      if (location_areas_.count(LocationType::LandMarkLocation)) {
        location_areas_.at(LocationType::LandMarkLocation).push_back(area_info);
      } else {
        std::vector<LocationArea> qr_vector = {area_info};
        location_areas_.insert(
            std::make_pair(LocationType::LandMarkLocation, qr_vector));
      }
    }

    if (navigation_model == "Laser") {
      area_info.min_mx =
          map_json["advancedAreaList"][i]["LeftTopPoint"]["x"].asDouble();
      area_info.max_my =
          map_json["advancedAreaList"][i]["LeftTopPoint"]["y"].asDouble();
      area_info.max_mx =
          map_json["advancedAreaList"][i]["RightBottomPoint"]["x"].asDouble();
      area_info.min_my =
          map_json["advancedAreaList"][i]["RightBottomPoint"]["y"].asDouble();
      SLAM_INFO(">>>>Laser_area is %f %f %f %f", area_info.min_mx,
                area_info.max_mx, area_info.min_my, area_info.max_my);
      if (location_areas_.count(LocationType::AMCL)) {
        location_areas_.at(LocationType::AMCL).push_back(area_info);
      } else {
        std::vector<LocationArea> qr_vector = {area_info};
        location_areas_.insert(std::make_pair(LocationType::AMCL, qr_vector));
      }
    }
  }
  location_module_->SetLocationArea(location_areas_);
  SLAM_INFO("load loaction area successful.......");
  return;
}

bool MappingAndLocationImpl::_loadChargePos(const Json::Value& map_json) {
  if (map_json["advancedPointList"].empty()) return false;
  int advanced_points_size = map_json["advancedPointList"].size();
  for (int i = 0; i < advanced_points_size; i++) {
    if (map_json["advancedPointList"][i]["className"].asString() ==
        "ChargePoint") {
      initial_position_.mfX =
        map_json["advancedPointList"][i]["pos"]["x"].asFloat();
      initial_position_.mfY =
        map_json["advancedPointList"][i]["pos"]["y"].asFloat();
      initial_position_.mfTheta =
        map_json["advancedPointList"][i]["pos"]["dir"].asFloat();
      SLAM_INFO("get charge pos %f %f %f", initial_position_.mfX,
                initial_position_.mfY, initial_position_.mfTheta);
      return true;
    }
  }
  SLAM_INFO("can not get ChargePos, please check map info");
  return false;
}


void MappingAndLocationImpl::_checkLadarMessage(
  const RadarSensoryMessage& ladar_message) {
  static int check_count = 0;
  if (check_count != 0) return;
  check_count++;
  float mfAngleMin =
    ladar_message.mstruRadarMessage.mstruRadarHeaderData.mfAngleMin;
  float mfAngleMax =
    ladar_message.mstruRadarMessage.mstruRadarHeaderData.mfAngleMax;
  if (mapping_and_location_config_.location_config.location_start_angle <
      mfAngleMin / M_PI * 180 ||
      mapping_and_location_config_.location_config.location_end_angle >
      mfAngleMax / M_PI * 180) {
      SLAM_ERROR("location param angle is more than ladar message (%f %f)",
                  mfAngleMin, mfAngleMax);
      return;
  }
  if (mapping_and_location_config_.mapping_config.mapping_start_angle <
      mfAngleMin / M_PI * 180 ||
      mapping_and_location_config_.mapping_config.mapping_end_angle >
      mfAngleMax / M_PI * 180) {
      SLAM_ERROR("mapping param angle is more than ladar message (%f %f)",
                  mfAngleMin, mfAngleMax);
      return;
  }
  float angle_increment =
    ladar_message.mstruRadarMessage.mstruRadarHeaderData.mfAngleIncreament;
  angle_increment = angle_increment / M_PI * 180;
  float location_laser_resolution =
    mapping_and_location_config_.location_config.location_laser_resolution;
  float mapping_laser_resolution =
    mapping_and_location_config_.mapping_config.laser_resolution;
  int times = location_laser_resolution / angle_increment;
  if (fabs(times * angle_increment - location_laser_resolution) > 1e-6) {
    SLAM_ERROR("location param ladar resolution %f diff angle_increment %f",
                  angle_increment, location_laser_resolution);
    return;
  }
  times = mapping_laser_resolution / angle_increment;
  if (fabs(times * angle_increment - mapping_laser_resolution) > 1e-6) {
    SLAM_ERROR("mapping param ladar resolution %f diff angle_increment %f",
                  angle_increment, mapping_laser_resolution);
    return;
  }
  SLAM_INFO("ladar param is right~~~~");
  return;
}



void MappingAndLocationImpl::_startMapping() {
  if (SYSTEM_STATE_MAPPING == device_state_->mcChassisState) {
    SLAM_ERROR("mapping order error, now is mapping....");
    return;
  } else if (SYSTEM_STATE_LOCATING == device_state_->mcChassisState ||
             SYSTEM_STATE_INIT_POSE == device_state_->mcChassisState) {
    SLAM_ERROR("mapping order error, now is location.....");
    return;
  }

  device_state_->mcChassisState = SYSTEM_STATE_MAPPING;
  is_stop_mapping_ = false;
  mapping_module_->StartMapping();
  _publishRobotState();
  location_module_->ResetPose();
}

void *MappingAndLocationImpl::_stopMappingThread(void *ptr) {
  pthread_detach(pthread_self());
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(ptr);
  if (SYSTEM_STATE_MAPPING == lp_this->device_state_->mcChassisState) {
    SLAM_INFO("start stop mapping.....");
    usleep(500000);
    lp_this->mapping_module_->StopMapping(lp_this->save_map_name_);
    lp_this->device_state_->mcChassisState = SYSTEM_STATE_FREE;
    lp_this->_publishRobotState();
    lp_this->mapping_module_->SetQrCalibrationFlag(false);
    SLAM_INFO("end stop mapping.....");
  } else {
    SLAM_WARN("is not in mapping state....");
  }
  pthread_exit(NULL);
}

void* MappingAndLocationImpl::_mergeMappingThread(void* ptr) {
  pthread_detach(pthread_self());
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(ptr);
  SLAM_INFO("begin merge map");
  Json::Value data_value = lp_this->merge_map_value_;
  if (lp_this->mapping_and_location_config_.mapping_type ==
      MappingType::Trilateral_Mapping) {
    JointMap::JointMapType type = JointMap::Reflector;
    std::shared_ptr<JointMap::JointMapInterface> joint_map_tool =
      CreateJointMap(type,
      lp_this->mapping_and_location_config_.mapping_config);
    std::string major_map, second_map, final_map;
    major_map = data_value["map1name"].asString();
    second_map = data_value["map2name"].asString();
    std::vector<std::string> second_maps =
      internal_common::SplitCString(second_map, " ");
    final_map = data_value["map3name"].asString();
    joint_map_tool->SetMajorMapInfo(major_map);
    SLAM_INFO("major_map %s second_map %s final_map %s",
              major_map.c_str(), second_map.c_str(), final_map.c_str());
    for (int i = 0; i < second_maps.size(); i++) {
      if (i == 0) {
        joint_map_tool->SetSecondMap(second_maps[i]);
      } else {
        std::string map_name =
          lp_this->mapping_and_location_config_.
          mapping_config.map_data_file_path + "/" + second_maps[i];
        joint_map_tool->SetSecondMap(map_name);
      }
    }
    if (second_maps.empty()) {
      joint_map_tool->SetSecondMap(second_map);
    }
    if (!joint_map_tool->JointAndSaveMap(final_map)) {
      SLAM_ERROR("can no do joint, please check your maps~~~");
    }
  } else {
    JointMap::JointMapType type = JointMap::Qr;
    std::shared_ptr<JointMap::JointMapInterface> joint_map_tool =
      CreateJointMap(type,
      lp_this->mapping_and_location_config_.mapping_config);
    std::string major_map, second_map, final_map;
    major_map = data_value["MapName1"].asString();
    second_map = data_value["MapName2"].asString();
    final_map = data_value["MapName3"].asString();
    joint_map_tool->SetMajorMapInfo(major_map);
    joint_map_tool->SetSecondMap(second_map);
    if (!joint_map_tool->JointAndSaveMap(final_map)) {
      SLAM_ERROR("can no do joint, please check your maps~~~");
    }
  }
  SLAM_INFO("end merge map!!!!!!!!");
  pthread_exit(NULL);
}



void MappingAndLocationImpl::_publishRobotState() {
  // 状态改变
  gomros::common::Message<DeviceState> data;
  data.pack.mcChassisState = device_state_->mcChassisState;
  std::vector<char> output;
  data.ToCharArray(&output);
  state_publisher_->Publish(output.data(), output.size());
}

void *MappingAndLocationImpl::_publishRobotStateThread(void *ptr) {
  pthread_detach(pthread_self());
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(ptr);
  int pose_cnt = 0, state_cnt = 0;
  Position current_pos;
  int save_pos_count = 0;
  bool is_need_set_total = true;
  while (1) {
    if (!lp_this->is_emerge_) {
      lp_this->location_module_->DealEmerge();
    }
    if (lp_this->location_module_->FinishLocate()) {
      if (is_need_set_total) {
        is_need_set_total = false;
        lp_this->location_module_->SetTotalOdom(lp_this->total_odom_);
      }
      current_pos = lp_this->location_module_->GetCurrentPose();
      gomros::common::Message<gomros::message::TotalOdomMessgae>
          total_odom_and_pose;
      pthread_mutex_lock(&lp_this->last_odom_mutex_);
      total_odom_and_pose.pack.odom_oose = lp_this->last_odom_message_;
      pthread_mutex_unlock(&lp_this->last_odom_mutex_);
      total_odom_and_pose.pack.odom_oose.mclDeltaPosition = current_pos;
      total_odom_and_pose.pack.radar_pose.mclDeltaPosition =
          current_pos * lp_this->radar_pose_base_link_;
      total_odom_and_pose.pack.total_odom =
          lp_this->location_module_->GetTotalOdom();
      lp_this->total_odom_ = total_odom_and_pose.pack.total_odom;
      std::vector<char> out;
      total_odom_and_pose.ToCharArray(&out);
      lp_this->pose_publisher_->Publish(out.data(), out.size());
      gomros::common::Message<gomros::message::TotalOdomMessgae>
          qr_odom_and_pos;
      qr_odom_and_pos.pack.odom_oose.mclDeltaPosition = current_pos;
      qr_odom_and_pos.pack.total_odom = lp_this->location_module_->GetQrOdom();
      std::vector<char> qr_pos;
      qr_odom_and_pos.ToCharArray(&qr_pos);
      lp_this->qr_pos_publisher_->Publish(qr_pos.data(), qr_pos.size());
      pose_cnt++;
      if (pose_cnt > 20) {
        SLAM_INFO("current state is %d, current_pose (%f %f %f)",
              lp_this->device_state_->mcChassisState,
              current_pos.mfX, current_pos.mfY, current_pos.mfTheta);
        pose_cnt = 0;
      }
    } else {
      is_need_set_total = true;
    }
    state_cnt++;
    if (state_cnt >= 50 * 3) {
      lp_this->_publishRobotState();
      state_cnt = 0;
    }
    if (save_pos_count++ > 300) {
      save_pos_count = 0;
      lp_this->_savePoseToServer(current_pos);
    }
    usleep(20000);
  }
  pthread_exit(NULL);
}

void MappingAndLocationImpl::_robotValCallBackFunc(
  void *object, char *buf, const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  gomros::common::Message<gomros::message::SpeedMessage> speed_msg;
  speed_msg.FromCharArray(buf, size);
  // pthread_mutex_lock(&lp_this->robot_vel_mutex_);
  // lp_this->robot_line_vel_ = speed_msg.pack.tSpeed_.dVelLine;
  // lp_this->robot_angle_vel_ = speed_msg.pack.tSpeed_.dVelAng;
  // pthread_mutex_unlock(&lp_this->robot_vel_mutex_);
}

void MappingAndLocationImpl::_emergeCallBackFunc(void *object, char *buf,
                                                 const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  gomros::common::Message<gomros::message::IOsystemInputMessage> msg;
  msg.FromCharArray(buf, size);
  lp_this->is_emerge_ = msg.pack.input[2];
}

void MappingAndLocationImpl::_odomCallBackFunc(void *object, char *buf,
                                               const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  gomros::common::Message<OdometerMessage> odometer_data;
  odometer_data.FromCharArray(buf, size);
  // static int odom_count = 0;
  // if (odom_count++ % 1000 == 0) {
  //   odom_count = 0;
  //   SLAM_INFO("odom_message %f %f %f",
  //           odometer_data.pack.mstruSingleSteerSts.mdLinearVel,
  //           odometer_data.pack.mstruSingleSteerSts.mdAngularVel,
  //           odometer_data.pack.mstruSingleSteerSts.mdRotAngle);
  // }

  if (isnan(odometer_data.pack.mfAngularSpeed) ||
      isnan(odometer_data.pack.mfLinearSpeed) ||
      isnan(odometer_data.pack.mclDeltaPosition.mlTimestamp) ||
      odometer_data.pack.mclDeltaPosition.mlTimestamp == 0) {
    SLAM_ERROR("get odom data nan or error");
    return;
  }
  if (odometer_data.pack.mlDeltaTime <= 0) {
    if (lp_this->robot_stop_count_ < 65536) {
      lp_this->robot_stop_count_++;
    }
  } else {
    lp_this->robot_stop_count_ = 0;
  }
  if (lp_this->robot_stop_count_ > 70) {
    odometer_data.pack.mstruDiffSteerSts.mfLeftLinearVel = 0.0;
    odometer_data.pack.mstruDiffSteerSts.mfRightLinearVel = 0.0;
    lp_this->location_module_->SetIsMove(false);
    lp_this->is_move_ = false;
  } else {
    lp_this->location_module_->SetIsMove(true);
    lp_this->is_move_ = true;
  }
  pthread_mutex_lock(&lp_this->last_odom_mutex_);
  lp_this->last_odom_message_ = odometer_data.pack;
  pthread_mutex_unlock(&lp_this->last_odom_mutex_);

  if (lp_this->mapping_and_location_config_.mapping_config.
      record_config.is_need_record_data) {
    lp_this->record_sensor_data_->RecordOdomData(odometer_data.pack);
  }
  // 判断是否超出阈值
#ifndef TEST_DEBUG
  pthread_mutex_lock(&lp_this->robot_vel_mutex_);
  float delta_vel =
    fabs(odometer_data.pack.mfLinearSpeed) - fabs(lp_this->robot_line_vel_);
  float delta_ang =
   fabs(odometer_data.pack.mfAngularSpeed) - fabs(lp_this->robot_angle_vel_);
  if (delta_vel > 0.1 || delta_ang > 0.2) {
    SLAM_WARN("get robot speed is over send speed %f %f %f %f",
              lp_this->robot_line_vel_, odometer_data.pack.mfLinearSpeed,
              lp_this->robot_angle_vel_, odometer_data.pack.mfAngularSpeed);
    // pthread_mutex_unlock(&lp_this->robot_vel_mutex_);
    // return;
  }
  pthread_mutex_unlock(&lp_this->robot_vel_mutex_);
#endif
  if (lp_this->device_state_->mcChassisState == SYSTEM_STATE_MAPPING) {
    lp_this->location_module_->SetOdomData(odometer_data.pack);
    if (lp_this->is_stop_mapping_) return;
    lp_this->mapping_module_->HandleOdomData(odometer_data.pack);
  } else {
    lp_this->location_module_->SetOdomData(odometer_data.pack);
  }
}

void MappingAndLocationImpl::_radarCallBackFunc(void *object, char *buf,
                                                const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  RadarSensoryMessage radar_data;
  radar_data.from_char_array(buf, size);
  lp_this->_checkLadarMessage(radar_data);
  if (lp_this->mapping_and_location_config_.mapping_config.
      record_config.is_need_record_data) {
    lp_this->record_sensor_data_->RecordLadarData(radar_data, false);
  }
  if (lp_this->device_state_->mcChassisState == SYSTEM_STATE_MAPPING) {
    if (lp_this->is_stop_mapping_) return;
    lp_this->mapping_module_->HandleLaserData(radar_data, lp_this->is_move_);
  } else {
    lp_this->location_module_->SetRadarData(radar_data);
  }
}

void MappingAndLocationImpl::_imuCallBackFunc(void *object, char *buf,
                                              const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  gomros::common::Message<ImuSensoryMessage> imu_data;
  imu_data.FromCharArray(buf, size);
  if (lp_this->mapping_and_location_config_.mapping_config.
      record_config.is_need_record_data) {
    lp_this->record_sensor_data_->RecordImuData(imu_data.pack);
  }
  if (lp_this->robot_stop_count_ > 70) return;
  if (!lp_this->is_emerge_) {
    lp_this->location_module_->DealEmerge();
    return;
  }

  if (isnan(imu_data.pack.time_stamp) ||
      isnan(imu_data.pack.forward_linear_accel) ||
      isnan(imu_data.pack.z_angle) ||
      isnan(imu_data.pack.z_omega)) {
      SLAM_ERROR("imu data is nan");
  }
  if (lp_this->device_state_->mcChassisState == SYSTEM_STATE_MAPPING) {
    if (lp_this->is_stop_mapping_) return;
    lp_this->mapping_module_->HandleIMUData(imu_data.pack);
    lp_this->location_module_->SetImuData(imu_data.pack);
  } else {
    lp_this->location_module_->SetImuData(imu_data.pack);
  }
}

void MappingAndLocationImpl::_qrCallBackFunc(void *object, char *buf,
                                             const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  gomros::common::Message<DMreaderSensoryMessage> qr_data;
  qr_data.FromCharArray(buf, size);
  if (lp_this->mapping_and_location_config_.mapping_config.
      record_config.is_need_record_data) {
    lp_this->record_sensor_data_->RecordQrData(qr_data.pack);
  }

  if (lp_this->device_state_->mcChassisState == SYSTEM_STATE_MAPPING) {
    if (lp_this->is_stop_mapping_) return;
    lp_this->mapping_module_->HandleQRCodeData(qr_data.pack);
  } else {
    lp_this->location_module_->SetQrData(qr_data.pack);
  }
}

void MappingAndLocationImpl::_startLocateCallBack(void *object, char *buf,
                                                  const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  gomros::common::Message<Position> data;
  data.FromCharArray(buf, size);
  lp_this->initial_position_ = data.pack;
  SLAM_INFO("set initpos (%f %f %f)", lp_this->initial_position_.mfX,
            lp_this->initial_position_.mfY, lp_this->initial_position_.mfTheta);
  lp_this->device_state_->mcChassisState = SYSTEM_STATE_INIT_POSE;
  lp_this->_publishRobotState();
  lp_this->location_module_->SetHaveMap(false);
  lp_this->location_module_->SetHavePose(false);
  std::string file_name = lp_this->mapping_and_location_config_
                              .mapping_config.map_config_file_path +
                          "/" + "map_config.json";
  lp_this->_loadMapNameFromFile(file_name);
  if (lp_this->_loadMapDataFromFile(lp_this->map_file_full_path_)) {
    lp_this->location_module_->SetIniPose(data.pack);
    lp_this->location_module_->StartGlobalLocating();
  }
}

void MappingAndLocationImpl::_startMappingCallBack(void *object, char *buf,
                                                   const int size) {
  // SLAM_DEBUG("get start mapping cmd\n");
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
#ifdef TEST_DEBUG
  lp_this->device_state_->mcChassisState = SYSTEM_STATE_FREE;
  lp_this->_startMapping();
  return;
#endif
  CustomTaskString task_string;
  task_string.from_char_array(buf, size);
  Json::Reader reader;
  Json::Value data_json;
  std::string data_string;
  data_string = task_string.TaskString;
  // std::string(task_string.TaskString, task_string.str_len);
  SLAM_INFO("get start mapping cmd........");
  if (!reader.parse(data_string, data_json)) {
    SLAM_ERROR("parse mapping cmd failed.....");
    lp_this->mapping_module_->SetMappingConfig(
      lp_this->mapping_and_location_config_.mapping_config);
  } else {
    float distance_step = data_json["diatance_step"].asDouble();
    float angle_step = data_json["angle_step"].asDouble();
    int angle_range = data_json["angle_range"].asInt();
    float map_resolution = data_json["map_resolution"].asDouble();
    if (data_json["is_modify_param"].asBool()) {
      SLAM_INFO("change mappingandlocation param!!!!");
      MappingConfig mapping_config =
        lp_this->mapping_and_location_config_.mapping_config;
      mapping_config.mapping_start_angle = -angle_range / 2;
      mapping_config.record_config.mapping_start_angle = -angle_range / 2;
      SLAM_INFO("mapping_config.mapping_start_angle %f", -angle_range / 2);
      mapping_config.mapping_end_angle = angle_range / 2;
      mapping_config.record_config.mapping_end_angle = angle_range / 2;
      SLAM_INFO("mapping_config.mapping_end_angle %f", angle_range / 2);
      mapping_config.mapping_resolution = map_resolution;
      SLAM_INFO("mapping_config.map_resolution %f", map_resolution);
      mapping_config.record_config.distance_step = distance_step;
      SLAM_INFO("distance_step %f", distance_step);
      mapping_config.record_config.angular_step = angle_step;
      SLAM_INFO("distance_step %f", angle_step);
      lp_this->mapping_module_->SetMappingConfig(mapping_config);
    } else {
      lp_this->mapping_module_->SetMappingConfig(
        lp_this->mapping_and_location_config_.mapping_config);
      SLAM_INFO("do not change mappingandlocation param~~~");
    }
    lp_this->_startMapping();
  }
}

void MappingAndLocationImpl::_stopMappingCallBack(void *object, char *buf,
                                                  const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  SLAM_INFO("get end mapping cmd\n");
  if (SYSTEM_STATE_MAPPING != lp_this->device_state_->mcChassisState) {
    SLAM_WARN("is not in mapping state....");
    return;
  }
#ifdef TEST_DEBUG
  lp_this->is_stop_mapping_ = true;
  MappingType type = lp_this->mapping_and_location_config_.mapping_type;
  if (type == MappingType::Fusion) {
    lp_this->save_map_name_ = "fusion_test_map";
  } else if (type == MappingType::Karto) {
    lp_this->save_map_name_ = "karto_test_map";
  } else if (type == MappingType::Optimize_Mapping) {
    lp_this->save_map_name_ = "optimize_test_map";
  } else if (type == MappingType::Reflector) {
    lp_this->save_map_name_ = "reflector_test_map";
  } else if (type == MappingType::Trilateral_Mapping) {
    lp_this->save_map_name_ = "Trilateral_test_map";
  } else {
    lp_this->save_map_name_ = "test_map";
  }

  pthread_create(&lp_this->mapping_thread_, NULL, lp_this->_stopMappingThread,
                 lp_this);
  return;
#endif
  CustomTaskString task_string;
  task_string.from_char_array(buf, size);
  Json::Reader reader;
  Json::Value data_json;
  std::string data_string;
  // data_string = std::string(task_string.TaskString, task_string.str_len);
  data_string = task_string.TaskString;
  // SLAM_DEBUG("###收到停止建图指令\n");
  if (!reader.parse(data_string, data_json)) {
    SLAM_WARN("###停止建图指令解析错误\n");
  } else {
    if (lp_this->is_stop_mapping_) {
      SLAM_ERROR("now is mapping, please wait ..........");
      return;
    }
    lp_this->save_map_name_ = data_json["map_name"].asString();
    // SLAM_INFO("set map name is %s", lp_this->save_map_name_.c_str());
    // SLAM_INFO("start mapping thread>>>>>>>>>");
    lp_this->is_stop_mapping_ = true;
    pthread_create(&lp_this->mapping_thread_, NULL, lp_this->_stopMappingThread,
                   lp_this);
  }
}

void MappingAndLocationImpl::_calibrationQrCameraCallBack(
  void* object, char* buf, const int size) {
  return;
  MappingAndLocationImpl *lp =
    reinterpret_cast<MappingAndLocationImpl *>(object);
  CustomTaskString task_string;
  task_string.from_char_array(buf, size);
  Json::Reader reader;
  Json::Value data_json;
  std::string data_string;
  data_string = task_string.TaskString;
  if (!reader.parse(data_string, data_json)) {
    SLAM_ERROR("cmd error!!!!, please check~~~~~~~~");
  } else {
    int cmd = data_json["calibrate"].asInt();
    if (cmd == 1) {
      // 开始录制数据 -- 开始建图的逻辑一样
      SLAM_INFO("-----------start Calibration--------------");
      lp->mapping_module_->SetQrCalibrationFlag(true);
      lp->_startMapping();
    } else if (cmd == 2) {
      // 结束录制数据
      lp->save_map_name_ = data_json["map_name"].asString();
      lp->is_stop_mapping_ = true;
      pthread_create(&lp->mapping_thread_, NULL, lp->_stopMappingThread, lp);
      SLAM_INFO("-----------end Calibration, please write--------------");
    } else if (cmd == 3) {
      SLAM_INFO("-----------finish Calibration--------------");
    } else {
      SLAM_ERROR("cmd error!!!!, please check~~~~~~~~");
    }
  }
}


void MappingAndLocationImpl::_upDateMapCallBack(
  void* object, char* buf, const int size) {
  MappingAndLocationImpl *lp_this =
      reinterpret_cast<MappingAndLocationImpl *>(object);
  CustomTaskString task_string;
  task_string.from_char_array(buf, size);
  Json::Reader reader;
  Json::Value data_json;
  if (task_string.cmd_type == gomros::message::PUSH_MAP) {
    SLAM_INFO("get switch floor cmd ");
    std::string map_name;
    if (!reader.parse(task_string.TaskString, data_json)) {
      SLAM_ERROR("parse task string error!!!!");
      return;
    }
    map_name = std::string(data_json["MapName"].asString().data(),
                            data_json["MapName"].asString().size());
    lp_this->initial_position_.mfX = data_json["IniPosX"].asDouble();
    lp_this->initial_position_.mfY = data_json["IniPosY"].asDouble();
    lp_this->initial_position_.mfTheta = data_json["IniTheta"].asDouble();
    lp_this->device_state_->mcChassisState = SYSTEM_STATE_INIT_POSE;
    lp_this->_publishRobotState();
    lp_this->location_module_->SetHaveMap(false);
    lp_this->location_module_->SetHavePose(false);
    SLAM_INFO("switch floor map name is %s initial_position (%f %f %f)",
            map_name.c_str(), lp_this->initial_position_.mfX,
            lp_this->initial_position_.mfY, lp_this->initial_position_.mfTheta);
    if (lp_this->_loadMapDataFromFile(map_name)) {
      lp_this->location_module_->SetIniPose(lp_this->initial_position_, true);
      lp_this->location_module_->StartGlobalLocating();
    }
  } else if (task_string.cmd_type == gomros::message::MERGE_MAP) {
    SLAM_INFO("get joint map cmd!!!");

    if (!reader.parse(task_string.TaskString, data_json)) {
      SLAM_ERROR("parse task string error!!!!");
      return;
    }

    lp_this->merge_map_value_ = data_json;
    pthread_create(&lp_this->merge_map_thread_, NULL,
                    lp_this->_mergeMappingThread, lp_this);
  }
  return;
}


void MappingAndLocationImpl::_savePoseToServer(const Position& current_pose) {
  static int save_pos_index = 0;
  Json::Value zero_pose_json;
  Json::Value zero_pose;
  zero_pose["AgvX"] = current_pose.mfX;
  zero_pose["AgvY"] = current_pose.mfY;
  zero_pose["AgvTheta"] = current_pose.mfTheta;
  zero_pose["Total_odom"] = total_odom_;
  zero_pose_json["zero_pos"] = zero_pose;

  if (save_pos_index == 1) {
    std::string filename =
      mapping_and_location_config_.location_config.init_pose_file_path +
      "/zero.json";
    internal_common::Save(filename.c_str(), zero_pose_json);
    save_pos_index++;
  } else {
    std::string filename =
      mapping_and_location_config_.location_config.init_pose_file_path +
      "/zero1.json";
    internal_common::Save(filename.c_str(), zero_pose_json);
    save_pos_index = 1;
  }
}


}  // namespace mapping_and_location
}  // namespace data_process
}  // namespace gomros
