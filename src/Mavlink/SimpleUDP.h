// SPDX-License-Identifier: MIT
// Copyright (c) 2026 pschatzmann

#pragma once    
#include <WiFiUdp.h>

namespace mavlink_controller {

/**
 * @class SimpleUDP
 * @brief Extension of WiFiUDP that ensures MAVLink-compatible Stream behavior.
 *
 * This class extends WiFiUDP to provide the expected Stream interface for MAVLink and
 * MavlinkController. It ensures that:
 * - available() returns the size of the next UDP packet if the current one is consumed
 * - write() automatically wraps data in beginPacket()/endPacket() for each send
 *
 * Usage:
 *   Use SimpleUDP as a drop-in replacement for WiFiUDP when working with MAVLink or
 *   MavlinkController to ensure reliable packet handling and transmission.
 */
class SimpleUDP : public WiFiUDP {
 public:
  int available() override {
    int size = WiFiUDP::available();
    if (size == 0) {
      size = parsePacket();
    }
    MAV_DEBUG("SimpleUDP::available() = %d", size);
    return size;
  }
  size_t write(const uint8_t* buffer, size_t size) override {
    MAV_DEBUG("SimpleUDP::write(%d bytes)", size);
    beginPacket(remoteIP(), remotePort());
    size_t written = WiFiUDP::write(buffer, size);
    endPacket();
    return written;
  }
};

}  // namespace mavlink_controller