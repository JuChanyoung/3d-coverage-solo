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
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>

class uav
{
public:
	int index;
	double x,y,z,
	vx,vy,vz,
	ax,ay,az,
	qx,qy,qz,qw,
	wx,wy,wz,
	yaw;
	ros::Duration dt;
	mavros_msgs::State current_state;
	double zd = 1;

        mavros_msgs::PositionTarget ctrl_msg;
	//mavros_msgs::AttitudeTarget angular_msg;
        //geometry_msgs::TwistStamped angular_msg;
	ros::Time time;

        mavros_msgs::CommandLong cmd_msg;

	uav() {index = 0;};
	uav(int i) {index = i;};
	~uav() {};

	void ctrl(int i, const double & u, const double & v, const double & w, const double & yaw_rate)
	{
                ctrl_msg.header.stamp = ros::Time::now();
                ctrl_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
                ctrl_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                                     mavros_msgs::PositionTarget::IGNORE_PY |
                                     mavros_msgs::PositionTarget::IGNORE_PZ |
                                     mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::FORCE |
                                     mavros_msgs::PositionTarget::IGNORE_YAW |
                                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
		ctrl_msg.velocity.x = u;
		ctrl_msg.velocity.y = v;
		ctrl_msg.velocity.z = w;
		//ctrl_msg.yaw_rate = yaw_rate;

// /mavros/cmd/command

                dt = ros::Time::now()-time;
                cmd_msg.request.command = 115;
                cmd_msg.request.param1 += yaw_rate*.dt;
                time = ros::Time::now();


// /mavros/setpoint_position/local
                //angular_msg.twist.angular.z = yaw_rate;

// /mavros/setpoint_raw/attitude
/*
                //angular_msg.type_mask = 0b11111011; //dont know what this does yet
                angular_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | 
                                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                                        mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
                angular_msg.body_rate.z = yaw_rate;
                angular_msg.thrust = 1;
*/
	}

	void ctrlCallback (const geometry_msgs::TwistStamped & msg)
	{

		ctrl(index,
			msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z,msg.twist.angular.z);

			ROS_INFO("Matlab has control");

		}

		void stateCallback (const mavros_msgs::State::ConstPtr & msg)
		{

			current_state = *msg;

		}

	};
