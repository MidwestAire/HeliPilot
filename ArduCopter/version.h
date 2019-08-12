#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "HeliPilot V19.08.11"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 19,8,11,FIRMWARE_VERSION_TYPE_OFFICIAL

#define FW_MAJOR 19
#define FW_MINOR 8
#define FW_PATCH 11
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL
