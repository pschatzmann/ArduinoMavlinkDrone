
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 pschatzmann

#pragma once

// --- Core ---
#define MAVLINK_ALIGNED_FIELDS      1
#define MAVLINK_CRC_EXTRA           1
#define MAVLINK_COMMAND_24BIT       1

// --- Version ---
#define MAVLINK_VERSION             2
#define MAVLINK_PROTOCOL_VERSION    2

// --- Buffers ---
#define MAVLINK_COMM_NUM_BUFFERS    1
#define MAVLINK_MAX_PAYLOAD_LEN     255

// --- Identity ---
#define MAVLINK_SYSTEM_ID           42
#define MAVLINK_COMPONENT_ID        200

// --- Signing ---
#define MAVLINK_USE_SIGNING         0

// --- Convenience ---
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

// --- Logging ---
#define MAVLINK_MAX_LOG_SIZE       12

// --- ESP32 Logging ---
#define MAVLINK_USE_ESP_LOGGER      true

// ignore warnings about taking the address of packed struct members, which is
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
