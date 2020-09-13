/**
 *  A simple Drone which is controlled by mavlink messages. The functinliy of this class  
 *  is based on Dan Zimmermans [barebones_MAVLink](https://github.com/danzimmerman/barebones_MAVLink) example project.
 * 
 * We also implement a minimal ParameterStore which holds the parameter values which will be available to the application.
 *  
 */

#ifndef __SimpleMavlinkDrone_H__
#define __SimpleMavlinkDrone_H__

#include "arduino.h"
#include "mavlink.h"
#include "math.h"
#include "ParameterStore.h"

class SimpleMavlinkDrone {
    private:
        Stream *stream;
        Stream *logStreamPtr;
        ParameterStore *parameterStore;
        bool releaseParameterStore;
        mavlink_message_t mvl_tx_message; //A special MAVLink message data structure. 
        mavlink_message_t mvl_rx_message;
        mavlink_status_t mvl_rx_status;
        const uint8_t mvl_compid = 1; //Component ID and System ID identify us to QGroundControl
        const uint8_t mvl_sysid = 1;
        const uint8_t mvl_chan = MAVLINK_COMM_1;  //MAVLink channel 1 appears to be required at least for Blue Robotics QGC

        //In my actual code for the Cypress PSoC, I use timer interrupts to schedule all the robot tasks.
        const uint32_t hb_interval = 1000; //heartbeat interval in milliseconds - 1 second
        uint32_t t_last_hb = 0;
        const uint32_t sys_stat_interval = 100; //10 system status messages per second
        uint32_t t_last_sys_stat =0;
        int16_t sys_stat_count = 0;
        uint8_t mvl_armed = 0;
        uint8_t mvl_packet_received = 0;
        uint32_t timestamp = 0; // since system boot

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

        void log(char* str);
        void log(char* str, int num);

    public:
        /**
         *  Constructor
         */
        SimpleMavlinkDrone(Stream *str=NULL, ParameterStore *parameters=NULL);

        /**
         * Destructor 
         */
        ~SimpleMavlinkDrone();

        /**
         *  Returns a parameter value
         */
        virtual  float getValue(uint16_t id);
        
        /**
         *  Sets a parameter value
         **/
        virtual void setValue(uint16_t id, float value);

        /**
         *  Process messages
         **/
        virtual void loop();
        /**
         * It might be necessary to assgn the Drone to another stream 
         */
        virtual void reconnect(Stream *str);

        /**
         * Checks if the drone is armed
         */
        bool isArmed() {
            return mvl_armed==1;
        }

        /**
         *  Defines the stream which is used for logging
         */
        void setLog(Stream &log);


};

#endif
