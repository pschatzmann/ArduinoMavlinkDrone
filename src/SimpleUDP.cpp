/**
 * A Simple exension of the WiFiUDP class which makes sure that the basic Stream functioinaltiy
 * which is used by Mavlink is working as expected. 
 */

#include "SimpleUDP.h"

int SimpleUDP::available() {
    int size = WiFiUDP::available();
    // if the curren package is used up we prvide the info for the next
    if (size==0){
        size = parsePacket();
    }
    return size;
};

size_t SimpleUDP::write(const uint8_t *buffer, size_t size) {
    beginPacket(remoteIP(), remotePort());
    WiFiUDP::write(buffer,size);
    endPacket();
};


