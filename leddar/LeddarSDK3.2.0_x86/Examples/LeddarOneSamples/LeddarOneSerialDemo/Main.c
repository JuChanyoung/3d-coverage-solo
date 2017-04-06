// *****************************************************************************
// Module..: SerialDemo -- Software development kit for Leddar products. Sensor
//           communication demonstration program.
//
/// \file    Main.c
///
/// \brief   This is the main file containing the menu driver.
///
// Copyright (c) 2014 LeddarTech Inc. All rights reserved.
// Information contained herein is or may be confidential and proprietary to
// LeddarTech inc. Prior to using any part of the software development kit
// accompanying this notice, you must accept and agree to be bound to the
// terms of the LeddarTech Inc. license agreement accompanying this file.
// *****************************************************************************

#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include "LeddarOne.h"


#define MAX(x,y) (((x) > (y)) ? (x) : (y))


// Mapping between the value in the baud rate register and the real baud rate
// value
static int gBAUDS[] =
{
	115200, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
};

// *****************************************************************************
// Function: GetMenuChoice
//
/// \brief   Loops until a key is entered that is valid in a menu.
///
/// \return  The key entered.
// *****************************************************************************

static int
GetMenuChoice(void)
{
	int lResult = 0;

	while ((lResult < '1') || (lResult > '9'))
	{
		lResult = toupper(GetKey());

		if ((lResult == 'Q') || (lResult == 27))
		{
			break;
		}
	}

	return lResult;
}

// *****************************************************************************
// Function: DisplayDetections
//
/// \brief   Display detection as well as other information in a continuous
///          loop until the user presses a key.
// *****************************************************************************

static void
DisplayDetections(void* hConnection)
{
	LtAcquisition lAcquisition;

	puts("\nPress a key to start and then press a key again to stop.");
	GetKey();

	while (!KeyPressed())
	{
		if (LeddarOneGetResults(hConnection, &lAcquisition) == LT_SUCCESS)
		{
			int i;
			LtDetection *lDetections = lAcquisition.mDetections;

			printf("\nTimestamp    : %d\n", lAcquisition.mTimestamp);
			{
				printf("Temperature  : %.1f deg C\n", lAcquisition.mTemperature);
			}

			for (i = 0; i < lAcquisition.mDetectionCount; ++i)
			{
				printf("%7.3f %6.2f    ",
					   lDetections[i].mDistance, lDetections[i].mAmplitude);
			}

			puts("");
		}
		else
		{
			puts("Communication error, aborting.");
			break;
		}
	}

	// Absorb the key used to stop the loop.
	GetKey();
}

// *****************************************************************************
// Function: DisplayConfiguration
//
/// \brief   Display the current configuration parameters. What is displayed
///          is adapted for the type or sensor (assumed not called when
///          sensor does not support configuration).
// *****************************************************************************

static void
DisplayConfiguration(void* hConnection)
{
#define DC_COMM_ERROR "Communication Error!"

	LtU16 lValue;

	puts("");

	printf("Accumulation           : ");
	if (LeddarOneGetParameter(hConnection, LEDDARONE_CONFIG_ACCUMULATION, &lValue) == LT_SUCCESS)
	{
		printf("%d (%d)\n", lValue, 1 << lValue);
	}
	else
	{
		puts(DC_COMM_ERROR);
	}

	printf("Oversampling           : ");
	if (LeddarOneGetParameter(hConnection, LEDDARONE_CONFIG_OVERSAMPLING, &lValue) == LT_SUCCESS)
	{
		printf("%d (%d)\n", lValue, 1 << lValue);
	}
	else
	{
		puts(DC_COMM_ERROR);
	}

	printf("Base sample count      : ");
	if (LeddarOneGetParameter(hConnection, LEDDARONE_CONFIG_SAMPLE_COUNT, &lValue) == LT_SUCCESS)
	{
		printf("%d\n", lValue);
	}
	else
	{
		puts(DC_COMM_ERROR);
	}

	printf("LED power              : ");
	if (LeddarOneGetParameter(hConnection, LEDDARONE_CONFIG_LED_POWER, &lValue) == LT_SUCCESS)
	{
		printf("%d%%\n", lValue);
	}
	else
	{
		puts(DC_COMM_ERROR);
	}

	printf("Automatic LED intensity: ");
	if (LeddarOneGetParameter(hConnection, LEDDARONE_CONFIG_ACQ_OPTIONS, &lValue) == LT_SUCCESS)
	{
		printf("%s\n", (lValue & 0x0001) ? "Yes" : "No");
	}
	else
	{
		puts(DC_COMM_ERROR);
	}

	printf("Baud Rate              : ");
	if (LeddarOneGetParameter(hConnection, LEDDARONE_CONFIG_BAUD_RATE, &lValue) == LT_SUCCESS)
	{
		if (lValue < sizeof(gBAUDS) / sizeof(gBAUDS[0]))
		{
			printf("%d\n", gBAUDS[lValue]);
		}
		else
		{
			puts("??");
		}
	}
	else
	{
		puts(DC_COMM_ERROR);
	}

	printf("Modbus Address         : ");
	if (LeddarOneGetParameter(hConnection, LEDDARONE_CONFIG_MODBUS_ADDRESS, &lValue) == LT_SUCCESS)
	{
		printf("%d\n", lValue);
	}
	else
	{
		puts(DC_COMM_ERROR);
	}
}

// *****************************************************************************
// Function: SetBaudRate
//
/// \brief   Ask and set new baud rate
// *****************************************************************************

static void 
SetBaudRate(void* hConnection)
{
	int lChoice;

	do
	{
		puts("\nSelect baud rate:");
		puts("    1. 9600");
		puts("    2. 19200");
		puts("    3. 38400");
		puts("    4. 57600");
		puts("    5. 115200");

		lChoice = GetKey(); printf("%c\n", lChoice);
		if ((lChoice >= '1') && (lChoice <= '5'))
		{
			LtResult lResult = LeddarOneSetParameter(hConnection, LEDDARONE_CONFIG_BAUD_RATE, lChoice - '0');
			if (lResult == LT_SUCCESS)
			{
				break;
			}
			else
				printf("Operation failed (error %i)!\n", lResult);
		}
		else if (lChoice == 27)
		{
			break;
		}

	} while ((1));
}

// *****************************************************************************
// Function: SetAutoLedIntensity
//
/// \brief   Ask and set new LED intensity setting
// *****************************************************************************

static void 
SetAutoLedIntensity(void* hConnection)
{
	int lChoice;

	do
	{
		printf("\nUse automatic LED intensity [Y/N]? ");
		lChoice = toupper(GetKey()); printf("%c\n", lChoice);
		if ((lChoice == 'Y') || (lChoice == 'N'))
		{
			LtU16 uValue;
			LtResult lResult = LeddarOneGetParameter(hConnection, LEDDARONE_CONFIG_ACQ_OPTIONS, &uValue);
			if (lResult == LT_SUCCESS)
			{
				uValue &= ~0x0001;
				if (lChoice == 'Y') uValue |= 0x0001;

				lResult = LeddarOneSetParameter(hConnection, LEDDARONE_CONFIG_ACQ_OPTIONS, uValue);
				printf("Error %i setting acquisition options\n", lResult);
				break;
			}
			else
				printf("Error %i getting current acquisition options\n", lResult);
			break;
		}
		else if (lChoice == 27)
		{
			break;
		}

	} while ((1));
}

// *****************************************************************************
// Function: SetGenericParam
//
/// \brief   Ask and set value for a generic parameter
// *****************************************************************************

static void 
SetGenericParam(void* hConnection, LtU16 uParam)
{
	unsigned int uValue;

	printf("\nEnter new value: ");

	if (scanf("%u", &uValue) == 1)
	{
		LtResult lResult = LeddarOneSetParameter(hConnection, uParam, (LtU16)uValue);
		if (lResult != LT_SUCCESS) printf("Operation failed (error %i)!\n", lResult);
	}
	else
	{ 
		puts("Bad input; press a key..."); 
		GetKey(); 
	}
}

// *****************************************************************************
// Function: ConfigurationMenu
//
/// \brief   Display a menu to allow changing configuration parameters.
// *****************************************************************************

static void
ConfigurationMenu(void* hConnection)
{
	for (;;)
	{
		int      lChoice;
		LtResult lResult;

		puts("\n");
		puts("1. Change accumulation");
		puts("2. Change oversampling");
		puts("3. Change base sample count");
		puts("4. Change LED power");
		puts("5. Set automatic LED intensity");
		// Note that for a change to the baud rate or Modbus address to take
		// effect, the sensor must be restarted.
		puts("6. Change baud rate");
		puts("7. Change modbus address");
		puts("8. Write to permanent memory");
		puts("9. Exit");

		lChoice = GetMenuChoice();

		if ((lChoice == '9') || (lChoice == 'Q') || (lChoice == 27))
		{
			break;
		}

		switch (lChoice)
		{
			case '1':
				SetGenericParam(hConnection, LEDDARONE_CONFIG_ACCUMULATION);
				break;
			case '2':
				SetGenericParam(hConnection, LEDDARONE_CONFIG_OVERSAMPLING);
				break;
			case '3':
				SetGenericParam(hConnection, LEDDARONE_CONFIG_SAMPLE_COUNT);
				break;
			case '4':
				SetGenericParam(hConnection, LEDDARONE_CONFIG_LED_POWER);
				break;
			case '5':
				SetAutoLedIntensity(hConnection);
				break;
			case '6':
				SetBaudRate(hConnection);
				break;
			case '7':
				SetGenericParam(hConnection, LEDDARONE_CONFIG_MODBUS_ADDRESS);
				break;
			case '8':
				lResult = LeddarOneWriteConfiguration(hConnection);
				if (lResult != LT_SUCCESS) puts("Operation failed!");
				break;
		}
	}
}

// *****************************************************************************
// Function: GetSensorPortInfo
//
/// \brief   Ask the user how to connect to a sensor (COM port...).
// *****************************************************************************

static LtBool 
GetSensorPortInfo(const char* sPrompt, char sPort[LT_MAX_PORT_NAME_LEN], 
				  int* pAddress, unsigned int* pBaudRate, unsigned int* pParity, 
				  unsigned int* pStopBits)
{
	char sBaudRate[48] = { 0 };
	char sParams[48]   = { 0 };

#ifdef WIN32
	printf("%s [example: COM1 115200 8N1]: ", sPrompt);
#else
    printf("%s [example: ttyUSB0 115200 8N1]: ", sPrompt);
#endif
	if (scanf("%s %s %s", sPort, sBaudRate, sParams) < 1) return LT_FALSE;
	if (toupper(sPort[0]) == 'Q') return LT_FALSE;

	if (pBaudRate)
	{
		if (sBaudRate[0] != 0)
		{
			*pBaudRate = (unsigned)atoi(sBaudRate);
			if (*pBaudRate == 0) { puts("Bad baudrate specified");  return LT_FALSE; }
		}
		else
			*pBaudRate = 115200;
	}

	if (sParams[0] != 0)
	{
		if (sParams[0] != '8') { puts("We only currently support 8-bit byte size"); return LT_FALSE; }

		if (pParity)
		{
			switch (toupper(sParams[1]))
			{
				case 'N': *pParity = LT_PARITY_NONE; break;
				case 'E': *pParity = LT_PARITY_EVEN; break;
				case 'O': *pParity = LT_PARITY_ODD; break;
				default:
					puts("Bad parity specified"); return LT_FALSE;
			}
		}

		if (pStopBits)
		{
			*pStopBits = (unsigned)atoi(sParams+2);
			if ((*pStopBits == 0) || (*pStopBits > 2)) { puts("Bad stop bit count"); return LT_FALSE; }
		}
	}
	else
	{
		if (pParity) *pParity = LT_PARITY_NONE;
		if (pStopBits) *pStopBits = 1;
	}


	printf("Please enter the MODBUS address: ");
	if (scanf("%d", pAddress) != 1) return LT_FALSE;

	return LT_TRUE;
}

// *****************************************************************************
// Function: ConnectMenu
//
/// \brief   Display a menu of actions that can be performed when connected.
// *****************************************************************************

static void ConnectMenu(void)
{
	char  lBuffer[LT_MAX_PORT_NAME_LEN];
	int   lAddress;
	void* hConnection;
	unsigned int uBaudRate, uParity, uStopBits;
	LtResult result;

	if (!GetSensorPortInfo("\nPort name", lBuffer, &lAddress, &uBaudRate,
		                   &uParity, &uStopBits))
	{
		return ;
	}

	result = LeddarOneConnect(&hConnection, lBuffer, lAddress, uBaudRate, uParity, uStopBits);
	if (result == LT_SUCCESS)
	{
		for (;;)
		{
			int lChoice;

			puts("\n\n1. Display detections");
			puts("2. Display configuration");
			puts("3. Change configuration");
			puts("4. Disconnect");

			lChoice = GetMenuChoice();

			switch (lChoice)
			{
				case '1':
					DisplayDetections(hConnection);
					break;
				case '2':
					DisplayConfiguration(hConnection);
					break;
				case '3':
					ConfigurationMenu(hConnection);
					break;

				case 'Q':
				case 27:
				case '4':
					LeddarOneDisconnect(hConnection);
					return;
			}
		}
	}
	else
	{
		printf("\nConnection failed (error %i)!\n", (int)result);
	}
}

// *****************************************************************************
// Function: SideBySideSensorsTest
//
/// \brief   Tests 2 sensors, displaying detections side by side
// *****************************************************************************

static void 
SideBySideSensorsTest()
{
	LtResult result;
	char  sSensor1Addr[LT_MAX_PORT_NAME_LEN];
	char  sSensor2Addr[LT_MAX_PORT_NAME_LEN];
	int   lAddress1;
	int   lAddress2;
	void* hConnection1 = NULL;
	void* hConnection2 = NULL;

	do // ONCE
	{
		puts("(Note: Defaulting to 115200 8N1.)");

		printf("\nPlease enter the port name of the first sensor ['q' cancel]: ");
		if (scanf("%s", sSensor1Addr) != 1) break;
		if (toupper(sSensor1Addr[0]) == 'Q') break;

		printf("Please enter the MODBUS address of the first sensor: ");
		if (scanf("%d", &lAddress1) != 1) break;

		printf("\nPlease enter the port name of the second sensor ['q' to cancel]: ");
		if (scanf("%s", sSensor2Addr) != 1) break;
		if (toupper(sSensor2Addr[0]) == 'Q') break;

		printf("Please enter the MODBUS address of the second sensor: ");
		if (scanf("%d", &lAddress2) != 1) break;

		result = LeddarOneConnect(&hConnection1, sSensor1Addr, lAddress1, 
								  115200, LT_PARITY_NONE, 1);
		if (result != LT_SUCCESS) { printf("Error %i connecting to the first sensor.", (int)result); break; }

		result = LeddarOneConnect(&hConnection2, sSensor2Addr, lAddress2, 
								  115200, LT_PARITY_NONE, 1);
		if (result != LT_SUCCESS) { printf("Error %i connecting to the second sensor.", (int)result); break; }

		// - - - - - - - - - - - - 
		// DISPLAY DETECTIONS
		// - - - - - - - - - - - - 
		LtAcquisition lAcquisition1;
		LtAcquisition lAcquisition2;

		puts("\nPress a key to start and then press a key again to stop.");
		GetKey();

		while (!KeyPressed())
		{
			result = LeddarOneGetResults(hConnection1, &lAcquisition1);
			if (result != LT_SUCCESS)
			{
				printf("Communication error (%i) on sensor 1, aborting.\n", (int)result);
				break;
			}

			result = LeddarOneGetResults(hConnection2, &lAcquisition2);
			if (result != LT_SUCCESS)
			{
				printf("Communication error (%i) on sensor 2, aborting.\n", (int)result);
				break;
			}

			printf("\nTimestamps: %-8d  |  %-8d\n", 
				   lAcquisition1.mTimestamp, lAcquisition2.mTimestamp);
			printf("Temperatures: %4.1f deg C  |  %4.1f deg C\n", 
				   lAcquisition1.mTemperature, lAcquisition2.mTemperature);

			for (int i = 0; i < MAX(lAcquisition1.mDetectionCount, lAcquisition2.mDetectionCount); ++i)
			{
				printf("Detection %i: ", i);
					   
				if (i < lAcquisition1.mDetectionCount)
				{
					printf("%7.3f %6.2f", lAcquisition1.mDetections[i].mDistance,
						   lAcquisition1.mDetections[i].mAmplitude);
				}
				else
					printf("--------------");

				printf("  |  ");
				if (i < lAcquisition2.mDetectionCount)
				{
					printf("%7.3f %6.2f", lAcquisition2.mDetections[i].mDistance, 
						   lAcquisition2.mDetections[i].mAmplitude);
				}
				else
					printf("--------------");

				puts("");
			}

			puts("");
		}

		// Absorb the key used to stop the loop.
		GetKey();

	} while ((0));

	if (hConnection2) LeddarOneDisconnect(hConnection2);
	if (hConnection1) LeddarOneDisconnect(hConnection1);
}


// *****************************************************************************
// Function: MainMenu
//
/// \brief   Display the main menu (connect or quit).
// *****************************************************************************

static void
MainMenu(void)
{
	for (;;)
	{
		int lChoice;

		puts("\n\n1. Connect");
		puts("2. Connect to two LeddarOne");
		puts("3. Quit");

		lChoice = GetMenuChoice();

		switch (lChoice)
		{
			case '1':
				ConnectMenu();
				break;

			case '2':
				SideBySideSensorsTest();
				break;

			case 27:
			case '3':
			case 'Q':
				return;
		}
	}
}

// *****************************************************************************
// Function: main
//
/// \brief   Standard C entry point!
// *****************************************************************************

int
main(int argc, char *argv[])
{
	puts("**********************************************************");
	puts("* Welcome to the LeddarOne Serial Demonstration Program! *");
	puts("**********************************************************");

	MainMenu();

	return 0;
}
