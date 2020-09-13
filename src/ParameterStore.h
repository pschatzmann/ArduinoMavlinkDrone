#ifndef __PARAMTER_STORE_H__
#define __PARAMTER_STORE_H__

/**
 * Since we can not foresee which parameters you want to manage, so we provide a flexible parameter
 * management base class which can easyly be extended or overwritten with your own implementation.
 */

#include "arduino.h"

#define THROTTLE            0  // from 0 to 1.0  
#define YAW                 1  // from -100.0 to 100.0 // Rudder 
#define PITCH               2  // from -100.0 to 100.0 // Elevator
#define ROLL                3  // from -100.0 to 100.0 // Aileron
#define VOLTAGE_BATTERY     4  // in milli volts
#define GPS_LATITUDE        5  
#define GPS_LONGITUDE       6  
#define GPS_ALTITUDE        7  
#define HEADING             8 // 0 - 360 deg  
#define RECEIVER_RSSI       9  


#define PARAMETER_COUNT    10

class ParameterStore {
    private:
        float* values;
        uint16_t size;

    public: 
        ParameterStore(uint16_t size = PARAMETER_COUNT);
        ~ParameterStore();
        virtual float getValue(uint16_t id);
        virtual void setValue(uint16_t id, float value);
};

#endif