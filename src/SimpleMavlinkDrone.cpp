#include "SimpleMavlinkDrone.h"

/**
 *  Constructor passing in a Stream which is used to read the data from. You can also
 *  use your own implementation of the parameter store
 */
SimpleMavlinkDrone::SimpleMavlinkDrone(Stream *stream, ParameterStore *parameterStore ) {
  log("SimpleMavlinkDrone");
  this->stream = stream;
  this->releaseParameterStore = parameterStore == NULL;
  this->parameterStore =  (parameterStore != NULL ) ? parameterStore : new ParameterStore();
}

SimpleMavlinkDrone::~SimpleMavlinkDrone() {
  log("~SimpleMavlinkDrone");
  if (releaseParameterStore)
    delete this->parameterStore;
}

float SimpleMavlinkDrone::getValue(uint16_t id){
  return this->parameterStore->getValue(id);
}

void SimpleMavlinkDrone::setValue(uint16_t id, float value){
  this->parameterStore->setValue(id, value);
}

void SimpleMavlinkDrone::reconnect(Stream *stream){
  log("reconnect");
  this->stream = stream;
}


//=========================== Main program is below ======================================


void SimpleMavlinkDrone::loop() {
   // timestamp since system boot
   timestamp = millis();

  /* ======================== Serial MAVLink Message Reception ====================
   * If bytes come in on the serial port, we feed them to mavlink_parse_char().
   * This helper function keeps track of incoming bytes and alerts us when it's 
   * received a complete, valid MAVLink message.
   * See https://github.com/mavlink/c_library_v2/blob/master/mavlink_helpers.h#L966
   *==============================================================================*/
   
  while (stream->available()) {
    uint8_t rxbyte = stream->read();
    mvl_packet_received = mavlink_parse_char(mvl_chan,rxbyte,  &mvl_rx_message, &mvl_rx_status);
    if (mvl_packet_received)
       break;
                                      
  }
  
  /* ====================== received MAVLink Message Handling ==================
   *  If a full incoming MAVLink message is received, AND the message 
   *  came from the GCS (System ID 255), we handle it here. 
   *  
   *  In this code we: 
   *  -Respond to initial messages sent from QGC to the vehicle when it first connects, including:
   *    -A parameter list request. We send an arbitary float parameter.
   *    -A request for "mission items." We say we have none.
   *    -A request for autopilot capabilities and version. We make some things up.
   *  -Take action on manual control joystick messages -- retransmitted back to QGC for viewing in MAVLink inspector
   *  -Arm or disarm the vehicle and acknowledge arm/disarm status based on incoming command messages.
   *  There may be more messages you want to handle. 
   *  The Message IDs below are defined in individual message's .h files.
   *  For example: https://github.com/mavlink/c_library_v2/blob/master/common/mavlink_msg_manual_control.h#L4
   *  You can find message numbers and field descriptions at https://mavlink.io/en/messages/common.html 
   * ==================================================================== */
  if ((mvl_packet_received) && (255==mvl_rx_message.sysid)) {
    processRequest();
  }

  /*=========================== Periodic Telemetry Transmissions ======================
   * To let QGroundControl know a vehicle is present, we send a heartbeat every 1s.
   * Here I'm using the millis() function to schedule outbound messages. 
   * You don't want to use blocking delay() calls in code like this, because you want
   * to read the incoming serial port as often as possible.
   * 
   * In my usual target system, I use timer interrupts for scheduling.
   * ==============================================================================*/
   
  if ((millis()-t_last_hb)>hb_interval) { //#0 HEARTBEAT https://mavlink.io/en/messages/common.html#HEARTBEAT
    processHeartbeat();
  }

  if ((millis()-t_last_sys_stat)>sys_stat_interval) {
    processStat();
  }
}


void SimpleMavlinkDrone::processRequest() {
  log("processRequest");
  mvl_packet_received = 0; //reset the "packet received" flag
  switch (mvl_rx_message.msgid)
  {
    case MAVLINK_MSG_ID_MANUAL_CONTROL: //#69 https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
    {
      handleManualControl(&mvl_rx_message);
      break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: //#21 https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST
    {
      handleParamRequestList(&mvl_rx_message);
      break;
    }
    case MAVLINK_MSG_ID_COMMAND_LONG: //#76 https://mavlink.io/en/messages/common.html#COMMAND_LONG
    {
      handleCommandLong(&mvl_rx_message);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: //#43 https://mavlink.io/en/messages/common.html#MISSION_REQUEST_LIST
    {
      handleMissionRequestList(&mvl_rx_message);
      break;
    }
    default:
      log("Unprocessed msgid:",mvl_rx_message.msgid);
  }
}

/*==============================================================
 * Message-handling and transmitting functions 
 * used in the main loop are defined below.
 *==============================================================*/
 
void SimpleMavlinkDrone::transmitMessage(mavlink_message_t* mvl_msg_ptr) {
  //log("transmitMessage");
  uint8_t tx_byte_buffer[512]={0}; //A byte buffer that will be sent from the serial port.
  uint16_t tx_buflen = mavlink_msg_to_send_buffer(tx_byte_buffer,mvl_msg_ptr);
  stream->write(tx_byte_buffer,tx_buflen);
}


// This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their
void SimpleMavlinkDrone::handleManualControl(mavlink_message_t* mvl_msg_ptr) {
  log("handleManualControl");
  mavlink_manual_control_t mvl_joy; //manual control data structure into which we decode the message
  mavlink_msg_manual_control_decode(mvl_msg_ptr,&mvl_joy);

 // Values are normalized to the range [-1000,1000] -> we devide it by 10 to convert it to %
 this->setValue(THROTTLE, 0.1f * mvl_joy.z);
 this->setValue(YAW, 0.1f * mvl_joy.r);
 this->setValue(PITCH, 0.1f * mvl_joy.x );
 this->setValue(ROLL, 0.1f * mvl_joy.y);

  //For now, let's just retransmit the manual control message to see it in MAVLink Inspector
  mavlink_msg_manual_control_encode_chan(mvl_sysid,mvl_compid,mvl_chan, &mvl_tx_message,&mvl_joy);
  transmitMessage(&mvl_tx_message);

}

void SimpleMavlinkDrone::handleParamRequestList(mavlink_message_t* mvl_msg_ptr) {
  log("handleParamRequestList");
  mavlink_param_value_t mvl_param;
  /*
  mvl_param.param_id[0] = 'a'; //a parameter ID string, less than 16 characters.
  mvl_param.param_id[1] = '_';
  mvl_param.param_id[2] = 'p';
  mvl_param.param_id[3] = 'a';
  mvl_param.param_id[4] = 'r';
  mvl_param.param_id[5] = 'm';
  mvl_param.param_id[6] = 0; //null terminated
  mvl_param.param_value = 123.456; //the parameter value as a float
  mvl_param.param_type = MAV_PARAM_TYPE_REAL32; //https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE
  mvl_param.param_count = 1; //We have just one parameter to send. 
  */
  mvl_param.param_count = 0;
  mvl_param.param_index = 0; 
  mavlink_msg_param_value_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                      &mvl_tx_message,&mvl_param);
  transmitMessage(&mvl_tx_message);
  
}

void SimpleMavlinkDrone::handleCommandLong(mavlink_message_t* mvl_msg_ptr) {
  log("handleCommandLong");
  mavlink_command_long_t mvl_cmd;
  mavlink_msg_command_long_decode(mvl_msg_ptr,&mvl_cmd);
  switch (mvl_cmd.command) {
    case (MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES): {
      if (1==mvl_cmd.param1)
      {
        mavlink_autopilot_version_t mvl_apv; https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION
        mvl_apv.flight_sw_version = 2;
        mvl_apv.middleware_sw_version = 1;
        mvl_apv.board_version = 1;
        mvl_apv.vendor_id = 10101;
        mvl_apv.product_id = 20202;
        mvl_apv.uid = 0;
        mvl_apv.capabilities = 0; //See: https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY
        mvl_apv.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;	
        mvl_apv.capabilities |= MAV_PROTOCOL_CAPABILITY_MAVLINK2;
        mavlink_msg_autopilot_version_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                                  &mvl_tx_message,&mvl_apv);
        transmitMessage(&mvl_tx_message);
      }
      break;
    }//end handling of autopilot capabilities request
    case (MAV_CMD_COMPONENT_ARM_DISARM): {
      mvl_armed = (1==mvl_cmd.param1) ? 1 : 0;

      //Acknowledge the arm/disarm command.
      mavlink_command_ack_t mvl_ack; //https://mavlink.io/en/messages/common.html#COMMAND_ACK
      mvl_ack.command = MAV_CMD_COMPONENT_ARM_DISARM; //https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
      mvl_ack.result = MAV_RESULT_ACCEPTED; //https://mavlink.io/en/messages/common.html#MAV_RESULT
      mavlink_msg_command_ack_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                          &mvl_tx_message,&mvl_ack);
      //skipped setting several fields here, with unknown consequences.
      transmitMessage(&mvl_tx_message);
      break;
    }//end handling of arm/disarm command
    default: {
      log("Unprocessed command: ", mvl_cmd.command);
    }
  }//end switch/case
}//end MVL_Handle_Command_Long()


void SimpleMavlinkDrone::handleMissionRequestList(mavlink_message_t* mvl_msg_ptr) {
  log("handleMissionRequestList");
  mavlink_mission_count_t mvl_mc;
  mvl_mc.target_system = mvl_sysid;
  mvl_mc.target_component = mvl_compid;
  mvl_mc.count = 0;
  mavlink_msg_mission_count_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                        &mvl_tx_message,&mvl_mc);
  transmitMessage(&mvl_tx_message);
}


void SimpleMavlinkDrone::processHeartbeat() {
  //log("processHeartbeat");

  mavlink_heartbeat_t mvl_hb; //struct with user fields: uint32_t custom_mode, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint8_t system_status;
  mvl_hb.type = MAV_TYPE_GENERIC; //My vehicle is an underwater ROV. Change as appropriate. See: https://github.com/mavlink/c_library_v2/blob/748192f661d0df3763501cfc432861d981952921/common/common.h#L69
  mvl_hb.autopilot = MAV_AUTOPILOT_GENERIC; //See https://github.com/mavlink/c_library_v2/blob/748192f661d0df3763501cfc432861d981952921/common/common.h#L40
  mvl_hb.system_status = MAV_STATE_ACTIVE;
  if (mvl_armed)  {
    mvl_hb.base_mode = MAV_MODE_MANUAL_ARMED;
  } else { 
    mvl_hb.base_mode = MAV_MODE_MANUAL_DISARMED;
  }
  mvl_hb.base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED; //We want to control the drone
  //mvl_hb.custom_mode=0xABBA; //custom mode, can be anything, I guess
  mavlink_msg_heartbeat_encode_chan(mvl_sysid,mvl_compid,mvl_chan,  &mvl_tx_message,&mvl_hb);
  transmitMessage(&mvl_tx_message);
  t_last_hb = millis();  
}


void SimpleMavlinkDrone::processStat() {
  //log("processStat");

  //I often use the mavlink_<message name>_pack_chan() functions that 
  //accept each field as an argument instead of the mavlink_<message name>_encode() that
  //accepts a struct. They should save some memory to skip the extra
  //message struct, but I think setting each field by name in a demo code is easier to follow.

  processStatSystem();
  processStatGPS();
  processStatRadioStatus();

  t_last_sys_stat = millis();  
}

void SimpleMavlinkDrone::processStatSystem() {
  //log("processStatSystem");
  mavlink_sys_status_t mvl_sys_stat; //#1 SYS_STATUS https://mavlink.io/en/messages/common.html#SYS_STATUS
  mvl_sys_stat.onboard_control_sensors_present = 0; //To set these, consult https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR
  mvl_sys_stat.onboard_control_sensors_enabled = 0;
  mvl_sys_stat.onboard_control_sensors_health = 0;
  mvl_sys_stat.load = 0;
  mvl_sys_stat.voltage_battery = getValue(VOLTAGE_BATTERY)*1000; //the only non-trivial telemetry we're sending, shows up several places in QGC
  mvl_sys_stat.current_battery = -1;
  mvl_sys_stat.battery_remaining = -1;
  mvl_sys_stat.drop_rate_comm = 0;
  mvl_sys_stat.errors_comm = 0;
  mvl_sys_stat.errors_count1 = 0;
  mvl_sys_stat.errors_count2 = 0;
  mvl_sys_stat.errors_count3 = 0;
  mvl_sys_stat.errors_count4 = 0;

  mavlink_msg_sys_status_encode_chan(mvl_sysid,mvl_compid,mvl_chan, &mvl_tx_message,&mvl_sys_stat);
  transmitMessage(&mvl_tx_message);
}

void SimpleMavlinkDrone::processStatGPS() {
  //log("processStatGPS");
  mavlink_global_position_int_t pos;
  pos.lat = getValue(GPS_LATITUDE)* 1E7;   // expressed as degrees * 1E7
  pos.lon = getValue(GPS_LONGITUDE)* 1E7;  // expressed as degrees * 1E7

  if (pos.lat!=0.0 || pos.lon!=0.0) {
    pos.alt = getValue(GPS_ALTITUDE) * 1000; // espressed in millimeters
    pos.time_boot_ms = timestamp;
    pos.hdg = getValue(HEADING); // undefined: UINT16_MAX;
    mavlink_msg_global_position_int_encode(mvl_sysid,mvl_compid, &mvl_tx_message,&pos);
    transmitMessage(&mvl_tx_message);
  }
}

void SimpleMavlinkDrone::processStatRadioStatus() {
  //log("processStatRadioStatus");
  mavlink_radio_status_t radio;
  radio.remrssi = getValue(RECEIVER_RSSI);
  radio.rssi = getValue(RECEIVER_RSSI);
  if (radio.remrssi!=0){
    mavlink_msg_radio_status_encode(mvl_sysid,mvl_compid, &mvl_tx_message,&radio);
    transmitMessage(&mvl_tx_message);
  }
}

void SimpleMavlinkDrone::setLog(Stream &log){
  logStreamPtr = &log;
}

void SimpleMavlinkDrone::log(char* str) {
  if(logStreamPtr)
    logStreamPtr->println(str);
}

void SimpleMavlinkDrone::log(char* str, int num) {
  if(logStreamPtr){
    logStreamPtr->print(str);
    logStreamPtr->println(num);
  }
}
