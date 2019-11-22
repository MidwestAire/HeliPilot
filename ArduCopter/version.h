#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "HeliPilot v20.01.01-dev"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 20,01,01,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 20
#define FW_MINOR 01
#define FW_PATCH 01
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL
