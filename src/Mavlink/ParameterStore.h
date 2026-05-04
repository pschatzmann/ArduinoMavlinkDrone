// SPDX-License-Identifier: MIT
// Copyright (c) 2026 pschatzmann

#pragma once
#include "MavlinkLogger.h"

#define THROTTLE 0         // from 0 to 1.0
#define YAW 1              // from -100.0 to 100.0 // Rudder
#define PITCH 2            // from -100.0 to 100.0 // Elevator
#define ROLL 3             // from -100.0 to 100.0 // Aileron
#define VOLTAGE_BATTERY 4  // in milli volts
#define GPS_LATITUDE 5
#define GPS_LONGITUDE 6
#define GPS_ALTITUDE 7
#define HEADING 8  // 0 - 360 deg
#define RECEIVER_RSSI 9

#define PARAMETER_COUNT 10

namespace mavlink_controller {

/**
 * @class ParameterStore
 * @brief Flexible parameter storage for MAVLink drone applications.
 *
 * This class provides a simple, extensible parameter storage mechanism for
 * drone applications. It manages an array of float values, indexed by parameter
 * ID, and can be extended or replaced with custom implementations as needed.
 *
 * Features:
 * - Dynamic allocation of parameter storage
 * - Safe access with bounds checking
 * - Easily extensible for custom parameter handling
 *
 * Usage:
 *   ParameterStore params;
 *   params.setValue(THROTTLE, 0.5f);
 *   float throttle = params.getValue(THROTTLE);
 */
class ParameterStore {
 public:
  ParameterStore(uint16_t size = PARAMETER_COUNT) {
    values = new float[size];
    this->size = size;
    for (int j = 0; j < size; j++) {
      values[j] = 0.0;
    }
  }
  ~ParameterStore() { delete[] values; }
  float getValue(uint16_t id) { 
    //MAV_DEBUG("ParameterMapStore::getValue(%d)", id);
    return (id < size) ? values[id] : 0.0; 
  }
  void setValue(uint16_t id, float value) {
    MAV_DEBUG("ParameterMapStore::setValue(%d,%f)", id, value);
    if (id < size) {
      values[id] = value;
    }
  }

 private:
  float* values = nullptr;
  uint16_t size;
};

}  // namespace mavlink_controller