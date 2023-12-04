/*
 * @Description: Copyright (C) 2023 山东亚历山大智能科技有限公司
 * @version: 1.2
 * @Author: renjy
 * @Data: 2023-09-05 22:56:03
 * @LastEditors: renjy
 * @LastEditTime: 2023-11-18 15:30:56
 */

#pragma once

#include <string>
#include <opencv2/opencv.hpp>



namespace DisPlayResult {

  // 原始雷达数据 并携带该帧雷达数据在全局坐标下的位姿信息
  const std::string raw_ladar_topic = "raw_ladar";

  /* 
    携带反光柱的信息 激光头携带了反光柱在全局坐标下的位姿  -- 定位
  */
  const std::string reflector_ladar_topic = "reflector_info";
  /* 
    携带ekf反光柱的信息
  */
  const std::string reflector_ekf_topic = "reflector_ekf_info";
  /*
    反光柱SLAM建图时 状态信息  -- 雷达信息
  */
  const std::string reflector_mapping_topic = "state_info";

  // 栅格建图
  const std::string grid_map_topic = "grid_map_info";

  // 反光柱类型
  enum ReflectorType {
    InState = 0,   // 已在状态空间
    NewInstate = 1,  // 新加入状态空间
    Identify = 2,  // 识别到的反光柱
    InMap = 3,      // 在地图中
    MatchInMap = 4   // 地图被匹配的
  };

  inline cv::Scalar GetScalarByType(const ReflectorType& type) {
    if (InState == type) {
      return cv::Scalar(255, 0, 0);   // 蓝色
    } else if (NewInstate == type) {
      return cv::Scalar(0, 0, 255);    // 红色
    } else if (Identify == type) {
      return cv::Scalar(0, 255, 0);  // 绿色
    } else if (InMap == type) {
      return cv::Scalar(0, 0, 0);    // 黑色
    } else if (MatchInMap == type) {
      return cv::Scalar(255, 255, 0);   // 青色
    } else {
      return cv::Scalar(192, 192, 192);   // 灰色
    }
  }



}  // namespace DisPlayResult






