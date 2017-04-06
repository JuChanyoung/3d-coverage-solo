// *****************************************************************************
// Module..: SerialDemo
//
/// \file    LeddarOne.h
///
/// \brief   Declarations for the LeddarOne layer of the demo.
///
// Copyright (c) 2015 LeddarTech Inc. All rights reserved.
// Information contained herein is or may be confidential and proprietary to
// LeddarTech inc. Prior to using any part of the software development kit
// accompanying this notice, you must accept and agree to be bound to the
// terms of the LeddarTech Inc. license agreement accompanying this file.
// *****************************************************************************

#ifndef _LEDDARONE_H_
#define _LEDDARONE_H_

#ifdef __cplusplus
extern "C"
{
#endif
   
#include "OS.h"

#define LEDDAR_MAX_DETECTIONS 3

// Register addresses for configuration parameters
#define LEDDARONE_CONFIG_ACCUMULATION   0
#define LEDDARONE_CONFIG_OVERSAMPLING   1
#define LEDDARONE_CONFIG_SAMPLE_COUNT   2
#define LEDDARONE_CONFIG_LED_POWER      4
#define LEDDARONE_CONFIG_ACQ_OPTIONS    6                    // Bit 0 = Auto LED intensity
#define LEDDARONE_CONFIG_BAUD_RATE      29
#define LEDDARONE_CONFIG_MODBUS_ADDRESS 30

/// Measurement:
typedef struct _LtDetection
{
    float mDistance;
    float mAmplitude;
} LtDetection;

/// In a single reading, the sensor can detect more than one objects.
typedef struct _LtAcquisition
{
    LtU32 mTimestamp;
    float mTemperature;
    LtU16 mDetectionCount;
    
    LtDetection mDetections[LEDDAR_MAX_DETECTIONS];
} LtAcquisition;

// Establishes a connection with a sensor.  The connection handle is returned in *pConnection.
//   sPortName: Should be 'COMXXX' on Windows. On linux, only provide the device name under /dev.
//   aAddress: Modbus address (default = 1)
//   uBaudRate: 9600..115200
//   uParity: LT_PARITY_XXXX
//   uStopBits: 1 or 2
LtResult LeddarOneConnect(void** pConnection, const char *sPortName, LtByte aAddress,
						  unsigned int uBaudRate, unsigned int uParity, unsigned int uStopBits);

// Disconnects from the sensor:
void LeddarOneDisconnect(void* hConnection);

// Returns a set of detections (one reading):
LtResult LeddarOneGetResults(void* hConnection, LtAcquisition *aAcquisition);

// Returns the value of a parameter:
LtResult LeddarOneGetParameter(void* hConnection, LtU16 aNo, LtU16 *aValue);

// Sets the value of a parameter:
LtResult LeddarOneSetParameter(void* hConnection, LtU16 aNo, LtU16 aValue);

// Applies any settings changed:
LtResult LeddarOneWriteConfiguration(void* hConnection);

#ifdef __cplusplus
	}
#endif

#endif
