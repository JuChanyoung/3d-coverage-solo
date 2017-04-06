// LeddarOneLibModbusDemo.cpp: Sample that uses the open source libmodbus library.
//

#include <stdio.h>
#include <string.h>
#include <thread>
#include "extern/libmodbus-3.0.6/src/modbus.h"
#include "HighResClock.hpp"


// ***********************************************************************************************
// MACROS
// ***********************************************************************************************

#define LEDDAR_MAX_DETECTIONS 3

#define GETS_S(str) { \
	fgets(str, sizeof(str), stdin); \
	if (strlen(str) > 0) str[strlen(str)-1] = 0; }

#ifndef _MSC_VER
	typedef unsigned int UINT;
#endif

#ifndef MAX
	#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#endif

// ***********************************************************************************************
// TYPES
// ***********************************************************************************************

struct SDetection
{
	double dDistance;						 // distance from the sensor, in meters
	double dAmplitude;						 // signal amplitude
};


// ***********************************************************************************************
// IMPLEMENTATION
// ***********************************************************************************************

static bool AskSlaveAddr(modbus_t* mb)
{
	char str[128];

	printf("Enter slave address: ");
	GETS_S(str); if (str[0] == 0) return false;

	if (modbus_set_slave(mb, atoi(str)) != 0)
	{
		printf("Error setting slave id\n");
		return false;
	}

	return true;
}

static bool ReadDetections(modbus_t* mb, SDetection tabDetections[LEDDAR_MAX_DETECTIONS],
						   UINT& uDetectionCount, UINT& uTimestamp)
{
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Read 10 registers from the address 20
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	uint16_t tabRegValues[32];
	int numRead = modbus_read_input_registers(mb, 20, 10, tabRegValues);
	if (numRead == 10)
	{
		uTimestamp = tabRegValues[0] + (tabRegValues[1] << 16);
		float fTemperature = (float)tabRegValues[2] / 256.f;
		uDetectionCount = tabRegValues[3] < LEDDAR_MAX_DETECTIONS ? tabRegValues[3] : LEDDAR_MAX_DETECTIONS;

		for (UINT i = 0; i < uDetectionCount; ++i)
		{
			tabDetections[i].dDistance = (double)tabRegValues[i * 2 + 4] / 1000.0;
			tabDetections[i].dAmplitude = (double)tabRegValues[i * 2 + 5] / 256.0;
		}

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


// Tests the reading rate:
static void TestPerformances(modbus_t* mb)
{
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
		printf("%.1lf ", (double)tabTimes[i].count()/1000000.0);
	}
	printf("\n");
	printf("Min reading time = %.1lf ms\n", (double)minTimeBetweenReadings.count()/1000000.0);
	printf("Max reading time = %.1lf ms\n", (double)maxTimeBetweenReadings.count()/1000000.0);
	printf("Reading rate = %.1lf readings/s\n", 
	        (double)NBR_READINGS / ((double)std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime).count()/1000.0));
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
			printf("#%i: Distance = %.3lfm    Amplitude = %.1lf\n", i + 1,
					tabDetections[i].dDistance, tabDetections[i].dAmplitude);
		}
	}
	else
		printf("Error reading registers, errno=%i\n", errno);
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

	bool bContinue = true;

	std::thread thread = std::thread([=,&bContinue] 
	{
		// Read detections:
		while (bContinue)
		{
			SDetection tabDetections1[LEDDAR_MAX_DETECTIONS];
			SDetection tabDetections2[LEDDAR_MAX_DETECTIONS];
			UINT uDetectionCount1, uDetectionCount2;
			UINT uTimestamp1, uTimestamp2;

			if (modbus_set_slave(mb, iSlaveAddr1) != 0)
			{
				printf("Error setting slave id\n");
				return;
			}

			if (!ReadDetections(mb, tabDetections1, uDetectionCount1, uTimestamp1))
			{
				printf("Communication error on sensor 1, aborting (%i).\n", errno);
				return;
			}

			if (modbus_set_slave(mb, iSlaveAddr2) != 0)
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

			printf("\nTimestamps: %-8d  |  %-8d\n", uTimestamp1, uTimestamp2);

			for (UINT i = 0; i < MAX(uDetectionCount1, uDetectionCount2); ++i)
			{
				printf("Detection %u: ", i);

				if (i < uDetectionCount1)
				{
					printf("%7.3lf %6.2lf", tabDetections1[i].dDistance,
							tabDetections1[i].dAmplitude);
				}
				else
					printf("--------------");

				printf("  |  ");

				if (i < uDetectionCount2)
				{
					printf("%7.3f %6.2f", tabDetections2[i].dDistance,
							tabDetections2[i].dAmplitude);
				}
				else
					printf("--------------");

				puts("");
			}

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
	uint8_t buffer[128];
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

		if (numReceived >= 56)
		{
			if ((buffer[0] == 1) && (buffer[1] == 0x11))
			{
				if ((buffer[2 + 50] == 10) && (buffer[2 + 51] == 0))
				{
					printf("Talking to a LeddarOne\n");
				}
				else
				{
					for (int i = 0; i < numReceived; ++i)
					{
						printf("  Byte %i = %i\n", i, (int)buffer[i]);
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

// Entry point
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
		printf("5. Quit\n");
		printf("\n");

		GETS_S(str); 
		if ((str[0] == 0) || (str[0] == '5')) break;

		switch (str[0])
		{
			case '1': TestConnection(mb); break;
			case '2': TestOneReading(mb); break;
			case '3': TestPerformances(mb); break;
			case '4': TestSideBySideSensors(mb); break;
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

