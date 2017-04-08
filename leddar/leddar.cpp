// Standard C++ libraries
#include <iostream>
//#include <stdio>
#include <string>
#include <unistd.h>

// Leddar API
#include "include/LeddarC.h"
#include "include/LeddarCommands.h"
#include "include/LeddarProperties.h"
#include "include/laserscan_t.hpp"

// LCM libraries
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm.h>

// Leddar specifications.
#define BEAM_COUNT 16
static double max_range;
static double field_of_view;

static void connect(LeddarHandle handler, const char* serial) {
    int code = LeddarConnect(handler, serial);

    // Use default device` if unspecified.
    if (serial[0] == '\0') {
        serial = "default";
    }

    if (code == LD_SUCCESS) {
        //ROS_INFO("Connected to %s", serial);
    } else {
        //ROS_FATAL("Failed to connect to %s with code: %d", serial, code);
    }
}

int main(int argc, char ** argv)
{
  using namespace std;
  using namespace Laser;

  laserscan_t msg;

  // Initialize and connect to sensor
  LeddarHandle handler = LeddarCreate();
  char * serial = "/dev/ttyUSB0";
  connect(handler,serial);
  LdDetection detections[BEAM_COUNT];
  unsigned int count = LeddarGetDetectionCount(handler);
  if (count > BEAM_COUNT) {
      count = BEAM_COUNT;
  }

  // Initialize lcm handler
  lcm::LCM lcm("udpm://239.255.255.255:7667?ttl=1");
  if(!lcm.good())
  return 1;

  while (1) {
    // Acquire detections from Leddar.
    LeddarGetDetections(handler, detections, count);

    // Transfer detections into msg
    for (int i=0;i<count;i++){
      // msg.ranges.push_back(detections[i].mDistance);
      // msg.intensities.push_back(detections[i].mAmplitude);
      msg.ranges[i] = detections[i].mDistance;
      msg.intensities[i] = detections[i].mAmplitude;
    }

    // Publish data
    lcm.publish("Sensor", &msg);
    usleep(1000);
  }

  //lcm_destroy(lcm);
  return 0;
}
