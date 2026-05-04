// SPDX-License-Identifier: MIT
// Copyright (c) 2026 pschatzmann

#pragma once
#include <stdint.h>
#include <map>
#include "MavlinkLogger.h"

namespace mavlink_controller {

/**
 * @class ParameterMapStore
 * @brief Parameter storage using std::map for dynamic parameter IDs.
 *
 * This implementation allows storing and retrieving parameter values
 * using a map, so you are not limited to a fixed set or range of IDs.
 */
class ParameterMapStore {
 public:
  ParameterMapStore() = default;
  ~ParameterMapStore() = default;

  float getValue(uint16_t id) const {
    //MAV_DEBUG("ParameterMapStore::getValue(%d)", id);
    auto it = values.find(id);
    return (it != values.end()) ? it->second : 0.0f;
  }

  void setValue(uint16_t id, float value) {
    MAV_DEBUG("ParameterMapStore::setValue(%d,%f)", id, value);
    values[id] = value;
  }

 private:
  std::map<uint16_t, float> values;
};

}  // namespace mavlink_controller