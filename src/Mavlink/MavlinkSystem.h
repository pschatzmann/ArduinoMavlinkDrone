// SPDX-License-Identifier: MIT
// Copyright (c) 2026 pschatzmann

#pragma once
#include "MavlinkConfig.h"
#include "common/mavlink_types.h"

/// MAVLink system variable definition 
static mavlink_system_t mavlink_system = {
	.sysid = MAVLINK_SYSTEM_ID,
	.compid = MAVLINK_COMPONENT_ID
};

/// Send a byte on the MAVLink channel. This is used by the MAVLink library to send bytes.
void comm_send_ch(mavlink_channel_t chan, uint8_t b);