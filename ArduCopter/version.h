#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "HeliPilot v20-pre-release"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 20,01,01,FIRMWARE_VERSION_TYPE_BETA

#define FW_MAJOR 20
#define FW_MINOR 01
#define FW_PATCH 01
#define FW_TYPE FIRMWARE_VERSION_TYPE_BETA
