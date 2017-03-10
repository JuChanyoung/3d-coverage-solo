// ROS-defined header
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

// Standard header
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cstdlib>

// Msg header
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

// Mavros msg
#include <mavros/mavros.h>
#include <mavros/utils.h>
#include <mavros/mavros_uas.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandLong.h>

#include "uav.h"
#include "msg_print.h"

int main(int argc, char **argv)
{
  const int n=1; // N agents in the system

  // Declare ROS variables
  ros::init(argc, argv, "GCS_Main");
  ros::NodeHandle nh;
  ros::Subscriber state_sub[n];
  ros::Publisher command_pub[n];
  //ros::Publisher angular_pub[n];
  ros::ServiceClient arming_client[n];
  ros::ServiceClient set_mode_client[n];
  ros::ServiceClient yaw_ctrl_client[n];
  ros::Subscriber cov_ctrl_sub[n];

  ros::AsyncSpinner spinner(8);

  // Declare an array of uav's
  uav cluster[n];
  for(int i=0;i<n;i++)
  {
    uav fly(i);
    cluster[i] = fly;
  }

  // Initialize ROS subscribers and publishers
  for(int i=0;i<n;i++)
  {
    cov_ctrl_sub[i] = nh.subscribe(cov_ctrl(i), 1, &uav::ctrlCallback, &cluster[i]); // Uncomment to integrate with MATLAB
    state_sub[i] = nh.subscribe(state(i), 1, &uav::stateCallback, &cluster[i]);
    command_pub[i] = nh.advertise<mavros_msgs::PositionTarget>(ctrlpub(i), 10);
    //angular_pub[i] = nh.advertise<mavros_msgs::AttitudeTarget>(angularpub(i), 10);
    //angular_pub[i] = nh.advertise<geometry_msgs::TwistStamped>(angularpub(i), 1);
    arming_client[i] = nh.serviceClient<mavros_msgs::CommandBool>(arming(i));
    set_mode_client[i] = nh.serviceClient<mavros_msgs::SetMode>(set_mode(i));
    yaw_ctrl_client[i] = nh.serviceClient<mavros_msgs::CommandLong>(command_long(i));
  }

  ros::Rate loop_rate(40);

  // Declare mode and arm_cmd variables
  mavros_msgs::SetMode guided_set_mode;
  guided_set_mode.request.custom_mode = "GUIDED";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request[] = {ros::Time::now(),ros::Time::now()};

  spinner.start();

  while (ros::ok())
  {
    // Check arming and flight mode statuses
    for (int i=0;i<n;i++) {
      if( cluster[i].current_state.mode != "GUIDED" &&
      (ros::Time::now() - last_request[i] > ros::Duration(5.0))){
        if( set_mode_client[i].call(guided_set_mode) &&
        guided_set_mode.response.success){
          ROS_INFO("SOLO %d: Guided enabled", i+1);
        }
        last_request[i] = ros::Time::now();
      } else {
        if( !cluster[i].current_state.armed &&
          (ros::Time::now() - last_request[i] > ros::Duration(5.0))){
            if( arming_client[i].call(arm_cmd) &&
            arm_cmd.response.success){
              ROS_INFO("SOLO %d: Vehicle armed", i+1);
            }
            last_request[i] = ros::Time::now();
          }
        }
      }
      
      // Publish control cmd from MATLAB
      for(int i=0;i<n;i++)
      {
        //cluster[i].ctrl(i,0,0,1,0); // Testing cmd publishers
        command_pub[i].publish(cluster[i].ctrl_msg);
        //angular_pub[i].publish(cluster[i].angular_msg);
        yaw_ctrl_client[i].call(cluster[i].cmd_msg);
      }

      ros::spinOnce();
      loop_rate.sleep();
    }

    ros::waitForShutdown();
    return 0;

  }
