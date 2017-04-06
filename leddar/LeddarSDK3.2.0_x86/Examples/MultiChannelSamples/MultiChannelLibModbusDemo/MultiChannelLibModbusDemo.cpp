// MultiChannelLibModbusDemo.cpp: Sample that uses the open source libmodbus library.
#include <stdio.h>
#include <string.h>
#include <thread>
#include <ctype.h>
#include "extern/libmodbus-3.0.6/src/modbus.h"
#include "HighResClock.hpp"

// ***********************************************************************************************
// MACROS
// ***********************************************************************************************

#define LEDDAR_MAX_DETECTIONS 1*16					 // limited to one per channel in this example

#define GETS_S(str) { \
	fgets(str, sizeof(str), stdin); \
	if (strlen(str) > 0) str[strlen(str)-1] = 0; }

#ifndef _MSC_VER
typedef unsigned int UINT;
#endif

#ifndef ARRAYSIZE
	#define ARRAYSIZE(arr) (sizeof(arr) / sizeof(arr[0]))
#endif

#ifndef MAX
#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#endif

// ***********************************************************************************************
// TYPES
// ***********************************************************************************************

struct SDetection
{
	UINT channel;							 // Channel number (0..15)
	double dDistance;						 // distance from the sensor, in meters
	double dAmplitude;						 // signal amplitude
	uint8_t flags;                           // Flags
};


// ***********************************************************************************************
// GLOBAL VARIABLES
// ***********************************************************************************************

// Current slave address being used (because modbus_send_raw_request() doesn't use the address set 
// by modbus_set_slave()):
static int g_iSlaveAddr = -1;

// True to use command 0x41 to fetch data; otherwise, we use the standard Modbus registers:
static bool g_bUse0x41 = true;


// ***********************************************************************************************
// IMPLEMENTATION
// ***********************************************************************************************

static bool SetSlaveAddr(modbus_t* mb, int iSlaveAddr)
{
	if (modbus_set_slave(mb, iSlaveAddr) != 0)
	{
		g_iSlaveAddr = -1;
		return false;
	}

	g_iSlaveAddr = iSlaveAddr;

	return true;
}

static bool AskSlaveAddr(modbus_t* mb)
{
	char str[128];

	printf("Enter slave address: ");
	GETS_S(str); if (str[0] == 0) return false;

	if (!SetSlaveAddr(mb, atoi(str)))
	{
		printf("Error setting slave id\n");
		return false;
	}

	return true;
}

static bool ReadDetectionsFast(modbus_t* mb, SDetection tabDetections[LEDDAR_MAX_DETECTIONS],
						       UINT& nbrDetections, UINT& uTimestamp)
{
	// This version of the ReadDetections function uses the 0x41 function code.
	// It reduces the overhead and is the recommended way of reading measurements.

	// Read 5 holding registers from address 1
	uint8_t rawReq[] = { (uint8_t)g_iSlaveAddr, 0x41 };
	uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];

	if (modbus_send_raw_request(mb, rawReq, ARRAYSIZE(rawReq)) < 0)
	{
		printf("Error sending command 0x41 (errno=%i)\n", errno);
		return false;
	}

	// We had to add support for 0x41 in 'compute_data_length_after_meta()' in "modbus.c".
	int iResponseLength = modbus_receive_confirmation(mb, rsp);
	if (iResponseLength < 3)
	{
		printf("Error receiving response for command 0x41 (length=%i, errno=%i)\n", iResponseLength, errno);
		return false;
	}
	
	nbrDetections = rsp[2];

	if (iResponseLength >= 3 + (signed)nbrDetections*5)
	{
		for (UINT u = 0; (u < nbrDetections) && (u < LEDDAR_MAX_DETECTIONS); ++u)
		{
			uint8_t* pDetection = rsp + 3 + 5*u;
			tabDetections[u].dDistance = (pDetection[0] + (pDetection[1] << 8)) / 100.0;
			tabDetections[u].dAmplitude = (pDetection[2] + (pDetection[3] << 8)) / 64.0;
			tabDetections[u].channel = pDetection[4] >> 4;
			tabDetections[u].flags = pDetection[4] & 0x0F;
		}

		if (iResponseLength >= 3 + (signed)nbrDetections*5 + 6)
		{
			uint8_t* pTrailer = rsp + 3 + 5*nbrDetections;
			uTimestamp = pTrailer[0] + (pTrailer[1] << 8) + (pTrailer[2] << 16) + (pTrailer[3] << 24);
		}
	}
	else
	{
		printf("Not enough data received (length=%i)\n", iResponseLength);
		return false;
	}

	return true;        // success
}

static bool ReadDetectionsReg(modbus_t* mb, SDetection tabDetections[LEDDAR_MAX_DETECTIONS],
						      UINT& nbrDetections, UINT& uTimestamp)
{
	// This version of the ReadDetections function uses the standard Modbus registers.
	nbrDetections = 0;

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Read 32+2 registers from address 14
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	const UINT NBR_REGISTERS = 32+2;
	uint16_t tabRegValues[NBR_REGISTERS];
	int numRead = modbus_read_input_registers(mb, 14, NBR_REGISTERS, tabRegValues);
	if (numRead == NBR_REGISTERS)
	{
		uTimestamp = tabRegValues[0] + (tabRegValues[1] << 16);

		for (UINT i = 0; i < 16; ++i)
		{
			tabDetections[i].channel = i;
			tabDetections[i].dDistance = (double)tabRegValues[2 + i] / 100.0;
			tabDetections[i].dAmplitude = (double)tabRegValues[18 + i] / 64.0;
			tabDetections[i].flags = 0;                // unknown with the registers method
		}

		nbrDetections = 16;                    // limit to the first detection (one per segment)

		//printf("Read %i registers:\n", numRead);
		//for (int i = 0; i < numRead; ++i)
		//{
		//	printf("    Register %-2i = %u\n", i, (unsigned int)tabRegValues[i]);
		//}
	}
	else
	{
		//printf("Error reading registers: numRead=%i, errno=%i\n", numRead, errno);
		return false;
	}

	return true;    // success
}

static bool ReadDetections(modbus_t* mb, SDetection tabDetections[LEDDAR_MAX_DETECTIONS],
						   UINT& nbrDetections, UINT& uTimestamp)
{
	// Read sensor detections, using the method selected by the user.
	return g_bUse0x41 ? ReadDetectionsFast(mb, tabDetections, nbrDetections, uTimestamp) :
		                ReadDetectionsReg(mb, tabDetections, nbrDetections, uTimestamp);
}

static void TestPerformances(modbus_t* mb)
{
	// Tests the reading rate:
	const int NBR_READINGS = 200;

	double dMinDistance = 9999999.0f;
	double dMaxDistance = 0.0f;
	HighResClock::duration minTimeBetweenReadings(999999999999);
	HighResClock::duration maxTimeBetweenReadings(0);
	HighResClock::time_point lastReadingTime;
	HighResClock::duration tabTimes[NBR_READINGS];
	UINT uLastTimestamp = 0;

	if (!AskSlaveAddr(mb)) return;

	printf("Measuring acquisition rate, please wait...\n");

	auto startTime = HighResClock::now();

	int iTestNumber = 0;
	while (iTestNumber < NBR_READINGS)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(5));       // W/o this delay, we get some reading errors (timeouts) at some reading rates

		SDetection tabDetections[LEDDAR_MAX_DETECTIONS];
		UINT uDetectionCount;
		UINT uTimestamp;

		if (ReadDetections(mb, tabDetections, uDetectionCount, uTimestamp))
		{
			if (iTestNumber > 0)
			{
				const HighResClock::duration timeElapsedSinceLastReading = HighResClock::now() - lastReadingTime;
				if (timeElapsedSinceLastReading < minTimeBetweenReadings) minTimeBetweenReadings = timeElapsedSinceLastReading;
				if (timeElapsedSinceLastReading > maxTimeBetweenReadings) maxTimeBetweenReadings = timeElapsedSinceLastReading;
				tabTimes[iTestNumber] = timeElapsedSinceLastReading;
			}

			if (uTimestamp != uLastTimestamp)
			{
				for (UINT i = 0; i < uDetectionCount; ++i)
				{
					if (tabDetections[i].dDistance > dMaxDistance) dMaxDistance = tabDetections[i].dDistance;
					if (tabDetections[i].dDistance < dMinDistance) dMinDistance = tabDetections[i].dDistance;
				}

				lastReadingTime = HighResClock::now();
				++iTestNumber;
				uLastTimestamp = uTimestamp;
			}
			else
				printf("=");
		}
		else
		{
			// No data available?
			printf(".");
		}
	}
	printf("\n");

	auto endTime = HighResClock::now();

	printf("Got %i readings\n", iTestNumber);
	printf("Min distance = %.1lf\n", dMinDistance);
	printf("Max distance = %.1lf\n", dMaxDistance);

	printf("Reading times: ");
	for (int i = 1; i < iTestNumber; ++i)			// no measurement at index 0
	{
		printf("%.1lf ", (double)tabTimes[i].count() / 1000000.0);
	}
	printf("\n");
	printf("Min reading time = %.1lf ms\n", (double)minTimeBetweenReadings.count() / 1000000.0);
	printf("Max reading time = %.1lf ms\n", (double)maxTimeBetweenReadings.count() / 1000000.0);
	printf("Reading rate = %.1lf readings/s\n",
		   (double)NBR_READINGS / ((double)std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count() / 1000.0));
}

static void TestOneReading(modbus_t* mb)
{
	// Shows the last reading.
	SDetection tabDetections[LEDDAR_MAX_DETECTIONS];
	UINT uDetectionCount;
	UINT uTimestamp;

	if (!AskSlaveAddr(mb)) return;

	if (ReadDetections(mb, tabDetections, uDetectionCount, uTimestamp))
	{
		printf("Timestamp: %u\n", uTimestamp);
		printf("Got %u detections\n", uDetectionCount);

		for (UINT i = 0; i < uDetectionCount; ++i)
		{
			printf("#%-2i: Segment %-2i   Distance = %.3lfm    Amplitude = %.1lf\n", i + 1, (UINT)tabDetections[i].channel,
				   tabDetections[i].dDistance, tabDetections[i].dAmplitude);
		}
	}
	else
		printf("Error reading detections, errno=%i\n", errno);
}

static void TestSideBySideSensors(modbus_t* mb)
{
	// Tests 2 sensors, displaying detections side by side
	char str[128];
	int iSlaveAddr1, iSlaveAddr2;

	printf("Enter the slave address of first sensor [empty=cancel]: ");
	GETS_S(str); if (str[0] == 0) return;
	iSlaveAddr1 = atoi(str);

	printf("Enter the slave address of second sensor [empty=cancel]: ");
	GETS_S(str); if (str[0] == 0) return;
	iSlaveAddr2 = atoi(str);

	printf("Press <ENTER> to start, and <ENTER> again to stop.");
	GETS_S(str); if (str[0] == 'q') return;

	puts("Showing the center detection for each sensor\n");

	bool bContinue = true;

	std::thread thread = std::thread([=, &bContinue]
	{
		// Read detections:
		while (bContinue)
		{
			SDetection tabDetections1[LEDDAR_MAX_DETECTIONS];
			SDetection tabDetections2[LEDDAR_MAX_DETECTIONS];
			UINT uDetectionCount1, uDetectionCount2;
			UINT uTimestamp1, uTimestamp2;

			if (!SetSlaveAddr(mb, iSlaveAddr1))
			{
				printf("Error setting slave id\n");
				return;
			}

			if (!ReadDetections(mb, tabDetections1, uDetectionCount1, uTimestamp1))
			{
				printf("Communication error on sensor 1, aborting (%i).\n", errno);
				return;
			}

			if (!SetSlaveAddr(mb, iSlaveAddr2))
			{
				printf("Error setting slave id\n");
				return;
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(5));

			if (!ReadDetections(mb, tabDetections2, uDetectionCount2, uTimestamp2))
			{
				printf("Communication error on sensor 2, aborting (%i).\n", errno);
				return;
			}

			if ((uDetectionCount1 >= 1) && (uDetectionCount2 >= 1))
			{
				printf("\nTimestamps: %-8d  |  %-8d\n", uTimestamp1, uTimestamp2);

				printf("%7.3lf %6.2lf", tabDetections1[uDetectionCount1 / 2].dDistance,
					   tabDetections1[uDetectionCount1 / 2].dAmplitude);

				printf("  |  ");

				printf("%7.3f %6.2f", tabDetections2[uDetectionCount2 / 2].dDistance,
					   tabDetections2[uDetectionCount2 / 2].dAmplitude);

				puts("");
			}
			else
				puts("No detection!");
		}
	});

	GETS_S(str);
	bContinue = false;

	thread.join();
}

static void TestConnection(modbus_t* mb)
{
	if (!AskSlaveAddr(mb)) return;

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// We send a MODBUS_SERVER_ID request to determine if we are talking to a Leddar sensor.
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	uint8_t buffer[256];
	buffer[0] = 1;
	buffer[1] = 0x11;		   // MODBUS_SERVER_ID

	if (modbus_send_raw_request(mb, buffer, 2) < 2)
	{
		printf("Error sending MODBUS_SERVER_ID request\n");
		return;
	}

	memset(buffer, 0, sizeof buffer);
	int numReceived = modbus_receive_confirmation(mb, buffer);
	if (numReceived >= 0)
	{
		printf("Received %i bytes from the sensor\n", numReceived);

		if (numReceived >= 155)
		{
			if ((buffer[0] == 1) && (buffer[1] == 0x11))
			{
				if ((buffer[2 + 150] == 9) && (buffer[2 + 151] == 0))
				{
					printf("Talking to a M16\n");
				}
				else if ((buffer[2 + 150] == 7) && (buffer[2 + 151] == 0))
				{
					printf("Talking to an evaluation kit\n");
				}
				else if ((buffer[2 + 50] == 10) && (buffer[2 + 51] == 0))
				{
					printf("Talking to a LeddarOne (please use the LeddarOne sample)\n");
				}
				else
				{
					for (int i = 0; i < numReceived; ++i)
					{
						printf("  Byte %i = %i ('%c')\n", i, (int)buffer[i], (buffer[i] >= 32) ? buffer[i] : ' ');
					}
				}
			}
			else
				printf("Unexpected answer (bad address or function)\n");
		}
		else
			printf("Unexpected answer\n");
	}
	else
		printf("Error receiving data from the sensor (%i)\n", errno);
}

int main(int argc, char* argv[])
{
	char str[128];
	char sSerialPort[128];

	printf("Enter serial port [empty=quit]: ");
	GETS_S(sSerialPort); if (sSerialPort[0] == 0) return 1;

	modbus_t* mb = nullptr;       // "handle" for the libmodbus library

	// Selects the serial modbus interface:
	mb = modbus_new_rtu(sSerialPort, 115200, 'N', 8, 1);
	if (mb == nullptr)
	{
		printf("Unable to create the libmodbus context\n");
		return 2;
	}

	//modbus_set_debug(mb, true);      // uncomment to view debug info

	// Connects to the sensor:
	if (modbus_connect(mb) != 0)
	{
		modbus_free(mb);
		printf("Connection error\n");
		return 3;
	}

	while ((true))
	{
		printf("\n");
		printf("1. Test connection\n");
		printf("2. Read one measurement\n");
		printf("3. Test performance\n");
		printf("4. Side-by-side acquisition\n");
		printf("5. Method to read detections: %s\n", g_bUse0x41 ? "FAST (0x41)" : "REGISTERS");
		printf("6. Quit\n");
		printf("\n");

		GETS_S(str);
		if ((str[0] == 0) || (str[0] == '6')) break;

		switch (str[0])
		{
			case '1': TestConnection(mb); break;
			case '2': TestOneReading(mb); break;
			case '3': TestPerformances(mb); break;
			case '4': TestSideBySideSensors(mb); break;
			case '5': g_bUse0x41 = !g_bUse0x41; break;
		}

		printf("\n");
	}

	if (mb)
	{
		modbus_close(mb);
		modbus_free(mb);
	}

	return 0;
}

