close all
rosshutdown
clear
clc
%----------------- Coverage parameters initilization --------------%

% Agents in the system
k = 1;

%----------------------- ROS initialization -----------------------%
setenv('ROS_MASTER_URI','http://dasc:11311/');
rosinit('NodeName','/Matlab');

% Global agent state variables

% Quaternion
global qx_1 qx_2;
global qy_1 qy_2;
global qz_1 qz_2;
global qw_1 qw_2;


% Position
global x_1 x_2;
global y_1 y_2;
global z_1 z_2;

% GPS coordinates
global long_1 long_2;
global lat_1 lat_2;
global alt_1 alt_2;

% IMU
global wx_1 wx_2 ax_1 ax_2;
global wy_1 wy_2 ay_1 ay_2;
global wz_1 wz_2 az_1 az_2;

% Uncomment if these are needed for filter
% 
% global orient_cov_1 orient_cov_2;
% orient_cov_1 = zeros(1,9);
% orient_cov_2 = zeros(1,9);
% 
% global pos_cov_1 pos_cov_2 type_1 type_2;
% pos_cov_1 = zeros(1,9);
% pos_cov_2 = zeros(1,9);
% 
% global angular_cov_1 angular_cov_2 accel_cov_1 accel_cov_2;
% angular_cov_1 = zeros(1,9);
% angular_cov_2 = zeros(1,9);
% accel_cov_1 = zeros(1,9);
% accel_cov_2 = zeros(1,9);

% Battery voltage
global volt_1 volt_2;

% Nodes list and callback functions
imuCallback_list = {@imuCallback_1 @imuCallback_2};
xyzCallback_list = {@xyzCallback_1 @xyzCallback_2};
statusCallback_list = {@statusCallback_1 @statusCallback_2};

for i=1:k
    xyz_sub(i) = rossubscriber(strcat(strcat('/mavros',num2str(i)),'/global_position/global'), 'sensor_msgs/NavSatFix',xyzCallback_list{i});
    pause(1);

    mav_state_sub(i) = rossubscriber(strcat(strcat('/mavros',num2str(i)),'/battery'),'mavros_msgs/BatteryStatus',statusCallback_list{i});
    pause(1)
    
    ctrl_pub(i) = rospublisher(sprintf('/cov_ctrl_%d',i),'geometry_msgs/TwistStamped');
    ctrl_msg(i) = rosmessage(ctrl_pub(i));
    pause(1);
    
    imu_sub(i) = rossubscriber(strcat(strcat('/mavros',num2str(i)),'/imu/data'),'sensor_msgs/Imu',imuCallback_list{i});
    pause(1);
end

rate = rosrate(60);

zd = 1;
kv = 0.5;

alt = [];
kk=1;
while 1  
   qw(kk)=qw_1;
   qx(kk)=qx_1;
   qy(kk)=qy_1;
   qz(kk)=qz_1;
      


    
    
    if (z_1<1)
   ctrl_msg(1).Twist.Linear.X = 0;
   ctrl_msg(1).Twist.Linear.Y = 0;
   ctrl_msg(1).Twist.Linear.Z = .2;
   ctrl_msg(1).Twist.Angular.Z = 3.14/4;
   fprintf('UP\n')
   z_1
    end
    if (z_1>1.1)
   ctrl_msg(1).Twist.Linear.X = 0;
   ctrl_msg(1).Twist.Linear.Y = 0;
   ctrl_msg(1).Twist.Linear.Z = -.2;
   ctrl_msg(1).Twist.Angular.Z = 3.14/4;
   fprintf('DOWN\n')
   z_1
    end
   
   drawnow
   send(ctrl_pub(1),ctrl_msg(1));
   kk=kk+1;
   waitfor(rate); 
    
end


rosshutdown;