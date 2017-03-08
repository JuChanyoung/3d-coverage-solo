#ifndef MSG
#define MSG
// ROS-defined header
#include "ros/ros.h"
#include "std_msgs/String.h"

// Standard header
#include <sstream>

char * ctrlpub( int i)
{

	switch(i)
	{
		case 0: return "/mavros1/setpoint_raw/local";
		case 1: return "/mavros2/setpoint_raw/local";
	}
}

char * angularpub( int i)
{

	switch(i)
	{
                  //case 0: return "/mavros1/setpoint_velocity/cmd_vel";
                  //case 1: return "/mavros2/setpoint_velocity/cmd_vel";
	          case 0: return "/mavros1/setpoint_raw/attitude";
		  case 1: return "/mavros2/setpoint_raw/attitude";
	}
}

char * cov_ctrl( int i)
{
	switch(i)
	{
		case 0: return "/cov_ctrl_1";
		case 1: return "/cov_ctrl_2";
	}
}

char * state( int i)
{
	switch(i)
	{
		case 0: return "/mavros1/state";
		case 1: return "/mavros2/state";
	}
}

char * arming( int i)
{
	switch(i)
	{
		case 0: return "/mavros1/cmd/arming";
		case 1: return "/mavros2/cmd/arming";
	}
}

char * set_mode( int i)
{
	switch(i)
	{
		case 0: return "/mavros1/set_mode";
		case 1: return "/mavros2/set_mode";
	}
}

char * command_long( int i)
{
	switch(i)
	{
		case 0: return "/mavros1/cmd/command";
		case 1: return "/mavros2/cmd/command";
	}
}

#endif
