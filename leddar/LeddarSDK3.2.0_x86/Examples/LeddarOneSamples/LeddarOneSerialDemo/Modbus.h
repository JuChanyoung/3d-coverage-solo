// *****************************************************************************
// Module..: SerialDemo
//
/// \file    Modbus.h
///
/// \brief   Declarations for the Modbus layer of the demo.
///
// Copyright (c) 2015 LeddarTech Inc. All rights reserved.
// Information contained herein is or may be confidential and proprietary to
// LeddarTech inc. Prior to using any part of the software development kit
// accompanying this notice, you must accept and agree to be bound to the
// terms of the LeddarTech Inc. license agreement accompanying this file.
// *****************************************************************************

#ifndef _MOBDUS_H_
#define _MOBDUS_H_

#include "OS.h"

#define MODBUS_MAX_PAYLOAD 252
#define MODBUS_SERVER_ID   0x11


typedef struct SModbusInfo_tag
{
	LtHandle handle;
	LtByte address;
	LtByte lastFunction;
} SModbusInfo;


// Returns true if connected:
LtBool ModbusConnected(SModbusInfo* pConnection);

// Sends a Modbus message:
LtResult ModbusSend(SModbusInfo* pConnection, LtByte aFunction, LtByte *aBuffer, LtByte aLength);

// Receive a response (assumes a Send was done before):
LtResult ModbusReceive(SModbusInfo* pConnection, LtByte *aBuffer);

// Tries to connect to a device on the given serial port:
LtResult ModbusConnect(SModbusInfo** ppConnection, const char *aPortName, LtByte aAddress,
					   unsigned int uBaudRate, unsigned int uParity, unsigned int uStopBits);

// Disconnects from the device. Has no effect if was not connected:
void ModbusDisconnect(SModbusInfo* pConnection);

// Implements the standard Modbus function 4 (but with only 1 register):
LtResult ModbusReadInputRegisters(SModbusInfo* pConnection, LtU16 aNo, LtU16 aCount, LtU16 *aValue);

// Implements the standard Modbus function 3 (but with only 1 register):
LtResult ModbusReadHoldingRegister(SModbusInfo* pConnection, LtU16 aNo, LtU16 *aValue);

// Implements the standard Modbus function 6:
LtResult ModbusWriteRegister(SModbusInfo* pConnection, LtU16 aNo, LtU16 aValue);

#endif
