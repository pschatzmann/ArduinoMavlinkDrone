// SPDX-License-Identifier: MIT
// Copyright (c) 2026 pschatzmann

#pragma once
#include <stdarg.h>
#include "Arduino.h"
#include "MavlinkConfig.h"

#if !defined(ESP32)
#undef MAVLINK_USE_ESPLOGGER
#define MAVLINK_USE_ESPLOGGER false
#endif


#if MAVLINK_USE_ESPLOGGER
#define MAV_ERROR(...)   ESP_LOGE("MAVLINK", __VA_ARGS__)
#define MAV_WARN(...)    ESP_LOGW("MAVLINK", __VA_ARGS__)
#define MAV_INFO(...)    ESP_LOGI("MAVLINK", __VA_ARGS__)
#define MAV_DEBUG(...)   ESP_LOGD("MAVLINK", __VA_ARGS__)

#else
namespace mavlink_controller {

/**
 * @class MavlinkLoggerClass
 * @brief Simple logger with log levels and configurable Print output (default
 * Serial).
 *
 * Usage:
 *   MavlinkLogger logger;
 *   logger.log(MavlinkLogger::INFO, "Hello %d", 42);
 *   logger.setOutput(&myPrint);
 */
class MavlinkLoggerClass {
 public:
  enum Level { ERROR = 0, WARN = 1, INFO = 2, DEBUG = 3 };

  MavlinkLoggerClass() = default;

  /// Set the Print output target (e.g., Serial)
  void setOutput(Print* out) { output = out; }

  /// Set the minimum log level
  void setLevel(Level lvl) { level = lvl; }

  /// Initialize logger with log level and output (default: INFO, Serial)
  bool begin(Level lvl = INFO, Print* out = &Serial) {
    setLevel(lvl);
    setOutput(out);
    return output != nullptr;
  }


  /// Log an error message (varargs, printf-style)
  size_t error(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    size_t res = log_va(ERROR, fmt, args);
    va_end(args);
    return res;
  }


  /// Log a warning message (varargs, printf-style)
  size_t warn(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    size_t res = log_va(WARN, fmt, args);
    va_end(args);
    return res;
  }

  /// Log an info message (varargs, printf-style)
  size_t info(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    size_t res = log_va(INFO, fmt, args);
    va_end(args);
    return res;
  }


  /// Log a debug message (varargs, printf-style)
  size_t debug(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    size_t res = log_va(DEBUG, fmt, args);
    va_end(args);
    return res;
  }

  /// Log a message with a specific log level (varargs, printf-style)
  size_t log(Level lvl, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    size_t res = log_va(lvl, fmt, args);
    va_end(args);
    return res;
  }

 private:
  Print* output = &Serial;
  Level level = INFO;

  // Helper for va_list logging
  size_t log_va(Level lvl, const char* fmt, va_list args) {
    if (lvl > level || !output) return 0;
    char buf[MAVLINK_MAX_LOG_SIZE];
    vsnprintf(buf, sizeof(buf), fmt, args);
    return output->println(buf);
  }
};

/// Global instance of the MavlinkLoggerClass
static MavlinkLoggerClass MavlinkLogger;

}

#define MAV_ERROR(...)   MavlinkLogger.error(__VA_ARGS__)
#define MAV_WARN(...)    MavlinkLogger.warn(__VA_ARGS__)
#define MAV_INFO(...)    MavlinkLogger.info(__VA_ARGS__)
#define MAV_DEBUG(...)   MavlinkLogger.debug(__VA_ARGS__)

#endif