/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.0
 * @Author: renjy
 * @Date: 2022-10-27 20:42:27
 * @LastEditTime: 2023-11-22 10:41:22
 */
#pragma once

#include <memory>
#include <string>
#include <sstream>

#include "common_lib/log.h"

#ifndef SLAM_ERROR
#define SLAM_ERROR(fmt, ...)                                             \
  gomros::common::SLAMLogger::GetInstance()->GetLogger()->WriteLog( \
  gomros::common::LOG_LEVEL::ERROR, __FILENAME__,                   \
  __LINE__, __FUNCTION__, fmt"\n", ##__VA_ARGS__)
#endif

#ifndef SLAM_INFO
#define SLAM_INFO(fmt, ...)                                              \
  gomros::common::SLAMLogger::GetInstance()->GetLogger()->WriteLog( \
  gomros::common::LOG_LEVEL::INFO, __FILENAME__,                    \
  __LINE__, __FUNCTION__, fmt"\n", ##__VA_ARGS__)
#endif

#ifndef SLAM_WARN
#define SLAM_WARN(fmt, ...)                                              \
  gomros::common::SLAMLogger::GetInstance()->GetLogger()->WriteLog( \
  gomros::common::LOG_LEVEL::WARNING, __FILENAME__,                 \
  __LINE__, __FUNCTION__, fmt"\n", ##__VA_ARGS__)
#endif

#ifndef SLAM_DEBUG
#define SLAM_DEBUG(fmt, ...)                                             \
  gomros::common::SLAMLogger::GetInstance()->GetLogger()->WriteLog( \
  gomros::common::LOG_LEVEL::DEBUG, __FILENAME__,                   \
  __LINE__, __FUNCTION__, fmt"\n", ##__VA_ARGS__)
#endif

namespace gomros {
namespace common {

// 封装日志打印信息
class SLAMLogger {
 public:
  static void SetLoggerConfig(const gomros::common::LOG_LEVEL& level,
            const std::string& logger_name, bool out_terminal);
  static SLAMLogger* GetInstance() {
    if (logger_ == nullptr) {
      logger_ = new SLAMLogger();
    }
    return logger_;
  }
  Logger* GetLogger() {
    if (p_logger_ == nullptr) {
      p_logger_ = new Logger(gomros::common::LOG_LEVEL::DEBUG,
                              "./MappingAndLoaction", true);
    }
    return p_logger_;
  }

 private:
  SLAMLogger();
  virtual ~SLAMLogger() {}
  // 禁止外部复制构造
  SLAMLogger(const SLAMLogger &signal) {}
  // 禁止外部赋值操作
  const SLAMLogger &operator=(const SLAMLogger &signal) {}
  // static std::recursive_mutex mutex_logger_data_;
  static SLAMLogger* logger_;
  static Logger* p_logger_;
};



class SLAMLoggerStream {
 public:
  SLAMLoggerStream(
    LOG_LEVEL level, const char* file, int line, const char* func)
      : level_(level), file_(file), line_(line), func_(func) {}

  ~SLAMLoggerStream() {
      SLAMLogger::GetInstance()->GetLogger()->WriteLog(
          level_, file_, line_, func_, "%s\n", stream_.str().c_str());
  }

  template <typename T>
  SLAMLoggerStream& operator<<(const T& msg) {
      stream_ << msg;
      return *this;
  }

 private:
  LOG_LEVEL level_;
  const char* file_;
  int line_;
  const char* func_;
  std::ostringstream stream_;
};

#ifndef SLAM_ERROR_LOG
#define SLAM_ERROR_LOG  gomros::common::SLAMLoggerStream(\
  gomros::common::LOG_LEVEL::ERROR, __FILE__, __LINE__, __FUNCTION__)
#endif

#ifndef SLAM_INFO_LOG
#define SLAM_INFO_LOG gomros::common::SLAMLoggerStream(\
  gomros::common::LOG_LEVEL::INFO, __FILE__, __LINE__, __FUNCTION__)
#endif

#ifndef SALM_WARN_LOG
#define SALM_WARN_LOG  gomros::common::SLAMLoggerStream(\
  gomros::common::LOG_LEVEL::WARNING, __FILE__, __LINE__, __FUNCTION__)
#endif

#ifndef SLAM_DEBUG_LOG
#define SLAM_DEBUG_LOG gomros::common::SLAMLoggerStream(\
  gomros::common::LOG_LEVEL::DEBUG, __FILE__, __LINE__, __FUNCTION__)
#endif


}  // namespace common
}  // namespace gomros
