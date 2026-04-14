/**
 *  A simple Drone which is controlled by mavlink messages. The functinliy of
 * this class is based on Dan Zimmermans
 * [barebones_MAVLink](https://github.com/danzimmerman/barebones_MAVLink)
 * example project.
 *
 * We also implement a minimal ParameterStore which holds the parameter values
 * which will be available to the application.
 *
 */

#ifndef __SimpleMavlinkDrone_H__
#define __SimpleMavlinkDrone_H__

#include "Arduino.h"
#include "ParameterStore.h"
#include "math.h"
#include "mavlink.h"

/**
 * @class SimpleMavlinkDrone
 * @brief A simple MAVLink drone controller for Arduino/ESP32.
 *
 * This class implements a minimal MAVLink drone interface, handling message reception,
 * parameter storage, and basic telemetry/command processing. It is based on Dan Zimmerman's
 * barebones_MAVLink example and provides hooks for custom parameter storage and logging.
 *
 * Features:
 * - Receives and parses MAVLink messages from a serial stream
 * - Handles basic MAVLink commands (manual control, parameter requests, mission requests, etc.)
 * - Periodically transmits heartbeat and telemetry messages
 * - Supports a user-defined callback for custom message handling before default processing
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
class SimpleMavlinkDrone {
 public:
  /// Callback type: returns bool, takes SimpleMavlinkDrone& and
  /// mavlink_message_t&
  typedef bool (*MessageCallback)(SimpleMavlinkDrone&, mavlink_message_t&);
  /// Constructor with Stream pointer and ParameterStore pointer
  SimpleMavlinkDrone(Stream* str = nullptr, ParameterStore* parameters = nullptr);
  /// Constructor with Stream reference
  SimpleMavlinkDrone(Stream& str = nullptr) : SimpleMavlinkDrone(&str, nullptrptr) {};

  /// Destructor
  ~SimpleMavlinkDrone();

  ///  Returns a parameter value
  virtual float getValue(uint16_t id);

  /// Sets a parameter value
  virtual void setValue(uint16_t id, float value);

  ///  Process messages
  virtual void loop();

  /// It might be necessary to assgn the Drone to another stream
  virtual void reconnect(Stream* str);

  /// Checks if the drone is armed
  bool isArmed() { return mvl_armed == 1; }

  /// Set the callback to be executed before processRequest in loop
  void setMessageCallback(MessageCallback cb);

  /// Defines the stream which is used for logging
  void setLog(Stream& log);

 private:
  MessageCallback messageCallback = nullptrptr;
  Stream* stream;
  Stream* logStreamPtr;
  ParameterStore* parameterStore;
  bool releaseParameterStore;
  mavlink_message_t mvl_tx_message;  // A special MAVLink message data
                                     // structure.
  mavlink_message_t mvl_rx_message;
  mavlink_status_t mvl_rx_status;
  const uint8_t mvl_compid =
      1;  // Component ID and System ID identify us to QGroundControl
  const uint8_t mvl_sysid = 1;
  const uint8_t mvl_chan =
      MAVLINK_COMM_1;  // MAVLink channel 1 appears to be required at least for
                       // Blue Robotics QGC

  // In my actual code for the Cypress PSoC, I use timer interrupts to schedule
  // all the robot tasks.
  const uint32_t hb_interval =
      1000;  // heartbeat interval in milliseconds - 1 second
  uint32_t t_last_hb = 0;
  const uint32_t sys_stat_interval =
      100;  // 10 system status messages per second
  uint32_t t_last_sys_stat = 0;
  int16_t sys_stat_count = 0;
  uint8_t mvl_armed = 0;
  uint8_t mvl_packet_received = 0;
  uint32_t timestamp = 0;  // since system boot

  /*==============================================================
   * Message-handling and transmitting functions
   * used in the main loop are defined below.
   *==============================================================*/

  virtual void transmitMessage(mavlink_message_t* mvl_msg_ptr);
  virtual void handleManualControl(mavlink_message_t* mvl_msg_ptr);
  virtual void handleParamRequestList(mavlink_message_t* mvl_msg_ptr);
  virtual void handleCommandLong(mavlink_message_t* mvl_msg_ptr);
  virtual void handleMissionRequestList(mavlink_message_t* mvl_msg_ptr);

  virtual void processRequest();
  virtual void processHeartbeat();

  virtual void processStat();
  virtual void processStatSystem();
  virtual void processStatGPS();
  virtual void processStatRadioStatus();

  void log(const char* str);
  void log(const char* str, int num);

};

#endif
