
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 pschatzmann

#pragma once

#include "Arduino.h"
#include "MavlinkConfig.h"  // must be included before mavlink.h to set the necessary defines
#include "MavlinkLogger.h"
#include "MavlinkSystem.h"
#include "ParameterStore.h"
#include "common/mavlink.h"
#include "math.h"

namespace mavlink_controller {
/**
 * @class MavlinkController
 * @brief A simple MAVLink drone controller for Arduino/ESP32.
 *
 * This class implements a minimal MAVLink drone interface, handling message
 * reception, parameter storage, and basic telemetry/command processing. It is
 * based on Dan Zimmerman's barebones_MAVLink example and provides hooks for
 * custom parameter storage and logging.
 *
 * Features:
 * - Receives and parses MAVLink messages from a serial stream
 * - Handles basic MAVLink commands (manual control, parameter requests, mission
 * requests, etc.)
 * - Periodically transmits heartbeat and telemetry messages
 * - Supports a user-defined callback for custom message handling before default
 * processing
 * - Provides a simple parameter store for application values
 * - Logging support via user-supplied stream
 *
 * Usage:
 *   1. Instantiate with a Stream and optional ParameterStore
 *   2. Call loop() regularly (e.g., in Arduino loop())
 *   3. Optionally set a message callback with setMessageCallback()
 *   4. Use setLog() to enable logging
 *
 * @note This class is designed for embedded use and avoids blocking operations.
 */

class MavlinkController {
 public:
  /// Callback type for MAVLink message handling
  typedef bool (*MessageCallback)(MavlinkController&, mavlink_message_t&);

  /// Constructor with optional Stream and ParameterStore
  MavlinkController(Stream* str = nullptr,
                    ParameterStore* parameters = nullptr) {
    MAV_INFO("MavlinkController");
    stream = str;
    releaseParameterStore = parameters == nullptr;
    parameterStore =
        (parameters != nullptr) ? parameters : new ParameterStore();
  }

  /// Constructor with Stream reference
  MavlinkController(Stream& str) : MavlinkController(&str, nullptr) {}

  /// Destructor
  virtual ~MavlinkController() {
    MAV_INFO("~MavlinkController");
    if (releaseParameterStore) delete parameterStore;
  }

  /// Get a parameter value by id
  virtual float getValue(uint16_t id) {
    if (!parameterStore) {
      MAV_ERROR("getValue: parameterStore is null");
      return 0.0f;
    }
    return parameterStore->getValue(id);
  }

  /// Set a parameter value by id
  virtual void setValue(uint16_t id, float value) {
    if (!parameterStore) {
      MAV_ERROR("setValue: parameterStore is null");
      return;
    }
    parameterStore->setValue(id, value);
  }

  /// Main loop: processes MAVLink messages and telemetry
  virtual void loop() {
    timestamp = millis();
    while (stream && stream->available()) {
      uint8_t rxbyte = stream->read();
      mvl_packet_received =
          mavlink_parse_char(mvl_chan, rxbyte, &mvl_rx_message, &mvl_rx_status);
      if (mvl_packet_received) {
        MAV_INFO("Received msgid: %d from sysid: %d", mvl_rx_message.msgid,
                 mvl_rx_message.sysid);
        break;
      }
    }
    if ((mvl_packet_received) && (mavlink_system_id == mvl_rx_message.sysid)) {
      bool handled = false;
      if (messageCallback) {
        handled = messageCallback(*this, mvl_rx_message);
      }
      if (!handled) {
        processRequest();
      }
    }
    if ((millis() - t_last_hb) > hb_interval) {
      processHeartbeat();
    }
    if ((millis() - t_last_sys_stat) > sys_stat_interval) {
      processStat();
    }
    // feed the dog
    delay(1);
  }

  /// Assign a new Stream for MAVLink communication
  virtual void reconnect(Stream* str) {
    MAV_INFO("reconnect");
    stream = str;
  }

  /// Returns true if the drone is armed
  bool isArmed() { return mvl_armed == 1; }

  /// Set the callback for custom MAVLink message handling
  void setMessageCallback(MessageCallback cb) { messageCallback = cb; }

  /// Defines the system ID for incoming MAVLink messages (1-255)
  bool setSystemId(int id) {
    if (id < 1 || id > 255) {
      MAV_ERROR("setSystemId: invalid system ID %d", id);
      return false;
    }
    mavlink_system_id = id;
    MAV_INFO("System ID set to %d", mavlink_system_id);
    return true;
  }
  
  /// Get the current system ID
  int getSystemId() const { return mavlink_system_id; }

 protected:
  static constexpr size_t TX_BUFFER_SIZE = 512;
  int mavlink_system_id = 255;  // Default system ID for incoming messages
  MessageCallback messageCallback = nullptr;
  Stream* stream = nullptr;
  ParameterStore* parameterStore = nullptr;
  bool releaseParameterStore = false;
  mavlink_message_t mvl_tx_message;
  mavlink_message_t mvl_rx_message;
  mavlink_status_t mvl_rx_status;
  const uint8_t mvl_compid = 1;
  const uint8_t mvl_sysid = 1;
  const uint8_t mvl_chan = MAVLINK_COMM_1;
  const uint32_t hb_interval = 1000;
  uint32_t t_last_hb = 0;
  const uint32_t sys_stat_interval = 100;
  uint32_t t_last_sys_stat = 0;
  int16_t sys_stat_count = 0;
  uint8_t mvl_armed = 0;
  uint8_t mvl_packet_received = 0;
  uint32_t timestamp = 0;

  /// Transmit a MAVLink message, returns bytes written or 0 on error
  virtual size_t transmitMessage(mavlink_message_t* mvl_msg_ptr) {
    if (!mvl_msg_ptr) {
      MAV_ERROR("transmitMessage: null message pointer");
      return 0;
    }
    if (!stream) {
      MAV_ERROR("transmitMessage: stream is null");
      return 0;
    }
    uint8_t tx_byte_buffer[TX_BUFFER_SIZE] = {0};
    uint16_t tx_buflen =
        mavlink_msg_to_send_buffer(tx_byte_buffer, mvl_msg_ptr);
    if (tx_buflen > TX_BUFFER_SIZE) {
      MAV_ERROR("transmitMessage: buffer overflow (%d)", tx_buflen);
      return 0;
    }
    return stream->write(tx_byte_buffer, tx_buflen);
  }

  virtual void handleManualControl(mavlink_message_t* mvl_msg_ptr) {
    MAV_INFO("handleManualControl");
    mavlink_manual_control_t mvl_joy;
    mavlink_msg_manual_control_decode(mvl_msg_ptr, &mvl_joy);
    setValue(THROTTLE, 0.1f * mvl_joy.z);
    setValue(YAW, 0.1f * mvl_joy.r);
    setValue(PITCH, 0.1f * mvl_joy.x);
    setValue(ROLL, 0.1f * mvl_joy.y);
    mavlink_msg_manual_control_encode_chan(mvl_sysid, mvl_compid, mvl_chan,
                                           &mvl_tx_message, &mvl_joy);
    transmitMessage(&mvl_tx_message);
  }

  virtual void handleParamRequestList(mavlink_message_t* mvl_msg_ptr) {
    MAV_INFO("handleParamRequestList");
    mavlink_param_value_t mvl_param;
    mvl_param.param_count = 0;
    mvl_param.param_index = 0;
    mavlink_msg_param_value_encode_chan(mvl_sysid, mvl_compid, mvl_chan,
                                        &mvl_tx_message, &mvl_param);
    transmitMessage(&mvl_tx_message);
  }

  virtual void handleCommandLong(mavlink_message_t* mvl_msg_ptr) {
    MAV_INFO("handleCommandLong");
    mavlink_command_long_t mvl_cmd;
    mavlink_msg_command_long_decode(mvl_msg_ptr, &mvl_cmd);
    switch (mvl_cmd.command) {
      case (MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES): {
        if (1 == mvl_cmd.param1) {
          mavlink_autopilot_version_t mvl_apv;
          mvl_apv.flight_sw_version = 2;
          mvl_apv.middleware_sw_version = 1;
          mvl_apv.board_version = 1;
          mvl_apv.vendor_id = 10101;
          mvl_apv.product_id = 20202;
          mvl_apv.uid = 0;
          mvl_apv.capabilities = 0;
          mvl_apv.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
          mvl_apv.capabilities |= MAV_PROTOCOL_CAPABILITY_MAVLINK2;
          mavlink_msg_autopilot_version_encode_chan(
              mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &mvl_apv);
          transmitMessage(&mvl_tx_message);
        }
        break;
      }
      case (MAV_CMD_COMPONENT_ARM_DISARM): {
        mvl_armed = (1 == mvl_cmd.param1) ? 1 : 0;
        mavlink_command_ack_t mvl_ack;
        mvl_ack.command = MAV_CMD_COMPONENT_ARM_DISARM;
        mvl_ack.result = MAV_RESULT_ACCEPTED;
        mavlink_msg_command_ack_encode_chan(mvl_sysid, mvl_compid, mvl_chan,
                                            &mvl_tx_message, &mvl_ack);
        transmitMessage(&mvl_tx_message);
        break;
      }
      default: {
        MAV_INFO("Unprocessed command: %d", mvl_cmd.command);
      }
    }
  }

  virtual void handleMissionRequestList(mavlink_message_t* mvl_msg_ptr) {
    MAV_INFO("handleMissionRequestList");
    mavlink_mission_count_t mvl_mc;
    mvl_mc.target_system = mvl_sysid;
    mvl_mc.target_component = mvl_compid;
    mvl_mc.count = 0;
    mavlink_msg_mission_count_encode_chan(mvl_sysid, mvl_compid, mvl_chan,
                                          &mvl_tx_message, &mvl_mc);
    transmitMessage(&mvl_tx_message);
  }

  virtual void processRequest() {
    MAV_INFO("processRequest: %d", mvl_rx_message.msgid);
    mvl_packet_received = 0;
    switch (mvl_rx_message.msgid) {
      case MAVLINK_MSG_ID_MANUAL_CONTROL: {
        handleManualControl(&mvl_rx_message);
        break;
      }
      case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
        handleParamRequestList(&mvl_rx_message);
        break;
      }
      case MAVLINK_MSG_ID_COMMAND_LONG: {
        handleCommandLong(&mvl_rx_message);
        break;
      }
      case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
        handleMissionRequestList(&mvl_rx_message);
        break;
      }
      default:
        MAV_INFO("Unprocessed msgid: %d", mvl_rx_message.msgid);
    }
  }

  virtual void processHeartbeat() {
    mavlink_heartbeat_t mvl_hb;
    mvl_hb.type = MAV_TYPE_GENERIC;
    mvl_hb.autopilot = MAV_AUTOPILOT_GENERIC;
    mvl_hb.system_status = MAV_STATE_ACTIVE;
    if (mvl_armed) {
      mvl_hb.base_mode = MAV_MODE_MANUAL_ARMED;
    } else {
      mvl_hb.base_mode = MAV_MODE_MANUAL_DISARMED;
    }
    mvl_hb.base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    mavlink_msg_heartbeat_encode_chan(mvl_sysid, mvl_compid, mvl_chan,
                                      &mvl_tx_message, &mvl_hb);
    transmitMessage(&mvl_tx_message);
    t_last_hb = millis();
  }

  virtual void processStat() {
    processStatSystem();
    processStatGPS();
    processStatRadioStatus();
    t_last_sys_stat = millis();
  }

  virtual void processStatSystem() {
    mavlink_sys_status_t mvl_sys_stat;
    mvl_sys_stat.onboard_control_sensors_present = 0;
    mvl_sys_stat.onboard_control_sensors_enabled = 0;
    mvl_sys_stat.onboard_control_sensors_health = 0;
    mvl_sys_stat.load = 0;
    mvl_sys_stat.voltage_battery = getValue(VOLTAGE_BATTERY) * 1000;
    mvl_sys_stat.current_battery = -1;
    mvl_sys_stat.battery_remaining = -1;
    mvl_sys_stat.drop_rate_comm = 0;
    mvl_sys_stat.errors_comm = 0;
    mvl_sys_stat.errors_count1 = 0;
    mvl_sys_stat.errors_count2 = 0;
    mvl_sys_stat.errors_count3 = 0;
    mvl_sys_stat.errors_count4 = 0;
    mavlink_msg_sys_status_encode_chan(mvl_sysid, mvl_compid, mvl_chan,
                                       &mvl_tx_message, &mvl_sys_stat);
    transmitMessage(&mvl_tx_message);
  }

  virtual void processStatGPS() {
    mavlink_global_position_int_t pos;
    pos.lat = getValue(GPS_LATITUDE) * 1E7;
    pos.lon = getValue(GPS_LONGITUDE) * 1E7;
    if (pos.lat != 0.0 || pos.lon != 0.0) {
      pos.alt = getValue(GPS_ALTITUDE) * 1000;
      pos.time_boot_ms = timestamp;
      pos.hdg = getValue(HEADING);
      mavlink_msg_global_position_int_encode(mvl_sysid, mvl_compid,
                                             &mvl_tx_message, &pos);
      transmitMessage(&mvl_tx_message);
    }
  }

  virtual void processStatRadioStatus() {
    mavlink_radio_status_t radio;
    radio.remrssi = getValue(RECEIVER_RSSI);
    radio.rssi = getValue(RECEIVER_RSSI);
    if (radio.remrssi != 0) {
      mavlink_msg_radio_status_encode(mvl_sysid, mvl_compid, &mvl_tx_message,
                                      &radio);
      transmitMessage(&mvl_tx_message);
    }
  }
};

/// Support legacy name for SimpleMavlinkDrone
using SimpleMavlinkDrone = MavlinkController;

}  // namespace mavlink_controller