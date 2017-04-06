#include <stdio>
#include <iostream>
#include <vector>

namespace laser
{
  class LaserScan {
  public:
    /*
    float32_t time_stamp;
    float32_t angle_min;
    float32_t angle_max;
    float32_t angle_increment;
    float32_t time_increment;
    float32_t scan_time;
    float32_t range_min;
    float32_t range_max;
    */
    std::vector< float32_t > ranges;
    std::vector< float32_t > intensities;
  };
}
