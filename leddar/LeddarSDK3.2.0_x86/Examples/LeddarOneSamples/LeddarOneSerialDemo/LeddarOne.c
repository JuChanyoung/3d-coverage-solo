// *****************************************************************************
// Module..: SerialDemo
//
/// \file    LeddarOne.c
///
/// \brief   Function definitions for the Leddar layer of the demo.
///
// Copyright (c) 2015 LeddarTech Inc. All rights reserved.
// Information contained herein is or may be confidential and proprietary to
// LeddarTech inc. Prior to using any part of the software development kit
// accompanying this notice, you must accept and agree to be bound to the
// terms of the LeddarTech Inc. license agreement accompanying this file.
// *****************************************************************************

#include <stdlib.h>
#include <string.h>

#include "LeddarOne.h"
#include "Modbus.h"

// *****************************************************************************
// Function: LeddarOneConnect
//
/// \brief   Try to connect to a sensor on the given serial port.
///
/// \param   pConnection  On return, set to the handle to use to call other functions
///
/// \param   sPortName  Name of serial port to open (e.g.: COM4 on Windows or
///                     ttyUSB0 on Linux).
/// \param   aAddress   The Modbus address of the sensor to talk to.
///
/// \param   uBaudRate  9600, 19200, 38400, 57600 or 115200...
///
/// \param   uParity    LT_PARITY_XXXX
///
/// \param   uStopBits  1 or 2
///
/// \return  LT_SUCCESS or any one of the LT error codes.
// *****************************************************************************

LtResult LeddarOneConnect(void** pConnection, const char* sPortName, LtByte aAddress,
						  unsigned int uBaudRate, unsigned int uParity, unsigned int uStopBits)
{
	LtResult lResult;

	lResult = ModbusConnect((SModbusInfo**)pConnection, sPortName, aAddress, 
							uBaudRate, uParity, uStopBits);
	if (lResult == LT_SUCCESS)
	{
		lResult = ModbusSend((SModbusInfo*)*pConnection, MODBUS_SERVER_ID, NULL, 0);

		if (lResult == LT_SUCCESS)
		{
			LtByte lId[MODBUS_MAX_PAYLOAD];

			lResult = ModbusReceive((SModbusInfo*)*pConnection, lId);
			if (lResult >= 0)
			{
				// Make sure we are talking to a Leddar One
				if ((lId[50] == 10) && (lId[51] == 0))
				{
					return LT_SUCCESS;
				}

				// Unrecognized device!
				lResult = LT_ERROR;
			}
		}

		ModbusDisconnect((SModbusInfo*)*pConnection);
	}

	return lResult;
}

// *****************************************************************************
// Function: LeddarOneDisconnect
//
/// \param   hConnection    Handle returned by LeddarConnect().
///
/// \brief   Disconnect. Has no effect if was not connected.
// *****************************************************************************

void LeddarOneDisconnect(void* hConnection)
{
	if (hConnection != NULL)
	{
		ModbusDisconnect(hConnection);
	}
}

// *****************************************************************************
// Function: LeddarOneGetResults
//
/// \brief   Read input registers to get the timestamp, temperature and echoes.
///
/// \param   hConnection  Handle returned by LeddarConnect().
///
/// \param   aAcquisition Pointer to array where detections will be written.
///
/// \return  LT_SUCCESS or any of the LT error codes.
// *****************************************************************************

LtResult LeddarOneGetResults(void* hConnection, LtAcquisition* aAcquisition)
{
	LtU16 lValues[10] = { 0 };

	LtResult lResult = ModbusReadInputRegisters(hConnection, 20, 10, lValues);

	if (lResult == LT_SUCCESS)
	{
		int i;
		LtDetection *lDetections = aAcquisition->mDetections;

		aAcquisition->mTimestamp = lValues[0] + (lValues[1] << 16);
		aAcquisition->mTemperature = lValues[2] / 256.f;
		aAcquisition->mDetectionCount = lValues[3] < LEDDAR_MAX_DETECTIONS ? lValues[3] : LEDDAR_MAX_DETECTIONS;

		for (i = 0; i < aAcquisition->mDetectionCount; ++i)
		{
			lDetections[i].mDistance = lValues[i * 2 + 4] / 1000.f;
			lDetections[i].mAmplitude = lValues[i * 2 + 5] / 256.f;
		}
	}

	return lResult;
}

// *****************************************************************************
// Function: LeddarOneGetParameter
//
/// \brief   Generic configuration parameter read access function (for integer
///          parameters).
///
/// \param   hConnection  Handle returned by LeddarConnect().
///
/// \param   aNo     Parameter number to read (one of the LEDDAR_CONFIG_*
///                  constants).
/// \param   aValue  Pointer to a variable that on output will contain the
///                  value if LT_SUCCESS is returned.
///
/// \return  LT_SUCCESS or any of the LT error codes.
// *****************************************************************************

LtResult LeddarOneGetParameter(void* hConnection, LtU16 aNo, LtU16 *aValue)
{
	return ModbusReadHoldingRegister(hConnection, aNo, aValue);
}

// *****************************************************************************
// Function: LeddarOneSetParameter
//
/// \brief   Generic configuration parameter write access function (for integer
///          parameters).
///
/// \param   hConnection  Handle returned by LeddarConnect().
///
/// \param   aNo     Parameter number to write (one of the LEDDAR_CONFIG_*
///                  constants).
/// \param   aValue  The new value to set.
///
/// \return  LT_SUCCESS or any of the LT error codes.
// *****************************************************************************

LtResult LeddarOneSetParameter(void* hConnection, LtU16 aNo, LtU16 aValue)
{
	return ModbusWriteRegister(hConnection, aNo, aValue);
}

// *****************************************************************************
// Function: LeddarOneWriteConfiguration
//
/// \brief   Ask the sensor to write all current configuration parameters to
///          permanent memory.
///
/// \param   hConnection  Handle returned by LeddarConnect().
///
/// \return  LT_SUCCESS or any of the LT error codes.
// *****************************************************************************

LtResult LeddarOneWriteConfiguration(void* hConnection)
{
	ModbusSend(hConnection, 0x46, NULL, 0);

	return ModbusReceive(hConnection, NULL);
}
