/**
 * A Simple exension of the WiFiUDP class which makes sure that the basic Stream functioinaltiy
 * which is used by the SimpleMavlinkDrone is working as expected:
 * - available needs to provide a value if there are additinal packages
 * - we add the necessary beginPacket and endPacket when we write data
 */
#ifndef __SIMPLEUPD_H__
#define __SIMPLEUPD_H__

#include <WiFiUdp.h>

class SimpleUDP : public WiFiUDP {

    /**
     * Provides the available size of the current package and if this is used up of the
     * next package
     */ 
    int available() override;

    /**
     *  Replys will be sent to the initial remote caller
     */
    size_t write(const uint8_t *buffer, size_t size) override;
};

#endif

