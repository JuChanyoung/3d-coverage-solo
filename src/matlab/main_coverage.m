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
%Tuning Parameters
c_star=50;
%worked okay was .0001 for xyz or 5e-8
K_u=5e-8; K_v=5e-8; K_w=5e-8; K_r=1e-9; K_s=1e-9;
r_i=24;
%New constants 5/19/16
lil_gam_1=10^-4;
lil_gam_2=10^-4;
lil_gam_3=10^-4;
lil_gam_4=10^-4;
lil_gam_5=10^-4;
Lam_0=0.5;
Lam_1=1;
cap_tol=10;
capture_count=0;
orient_tol=.3;
U_max=0.2;
%Until yaw problem is fixed let's make it large so we don't have to wait
%around all day for global coverage mode to find waypoints
%orient_tol=100;

%New on 5/19/16
Ep_star=1e8; %Code Test Value

%Activate this next line only to test global coverage
Ep_star=1e-100;

%Number of agents
k=1;
Global_Flag=zeros(k,1);

%set up coordinates
dx=4; dy=4; dz=4; droll=.5; dpitch=.5; dyaw=.5;
%set(gcf,'visible','off')
dt=.1;
time=0:dt:50000;
t_count=0;
v_count_20=0;
v_count_26=0;
v_count_32=0;

% Define search box
x_coord=[0:dx:200];
y_coord=[0:dy:200];
zee_coord=[-100:dz:0];
z_coord(1,1,:)=zee_coord;

roll_coord=[0:droll:2*pi];
pitch_coord=[0:dpitch:2*pi];
yaw_coord=[0:dyaw:2*pi];
% IC's will be a fixed position

x_i=zeros(k,length(time));
y_i=zeros(k,length(time));
z_i=zeros(k,length(time));
roll_i=zeros(k,length(time));
pitch_i=zeros(k,length(time));
yaw_i=zeros(k,length(time));
e_hat_save=ones(k,101);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Read quad positions from Vicon
% pos_data = importdata('cpp_pos.txt');
% pos_size = size(pos_data);
% 
% while pos_size(1)== 0
%     pos_data = importdata('cpp_pos.txt');
%     pos_size = size(pos_data);
% end
% 
% for kk=1:k
%     % X
%     if(pos_data(kk,1)<0)
%         pos_data(kk,1) = 0;
%     else if(pos_data(kk,1)>200)
%         pos_data(kk,1) = 200;
%         end% nFrames =length(time);
% 
% mov = struct('cdata', [],...
%                         'colormap', []);
% Title='Control';                  
% 
% vidObj = VideoWriter(Title);
% vidObj.FrameRate=10;
% open(vidObj)

%     end
%     
%     % Y
%     if(pos_data(kk,2)<0)
%         pos_data(kk,2) = 0;
%     else if(pos_data(kk,2)>200)
%         pos_data(kk,2) = 200;
%         end
%     end
%     
%     % Z
%     if(pos_data(kk,3)>0)
%         pos_data(kk,1) = 0;
%     else if(pos_data(kk,3)<-100)
%         pos_data(kk,3) = -100;
%         end
%     end
%      
%     x_i(kk,t_count+1)=pos_data(kk,1);
%     y_i(kk,t_count+1)=pos_data(kk,2);
%     z_i(kk,t_count+1)=pos_data(kk,3);
%     roll_i(kk,t_count+1)=roll_coord(11);
%     pitch_i(kk,t_count+1)=pitch_coord(11);
%     yaw_i(kk,t_count+1)=yaw_coord(11);
%     
% end
% 
% fid = fopen('cpp_pos.txt', 'wt');
% fclose(fid);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Q=zeros(length(x_coord),length(y_coord),length(z_coord));
%TEMP ICS
%roll_i(t_count+1)=0;
%pitch_i(t_count+1)=0;
%yaw_i(t_count+1)=0;
%x_i(t_count+1)=5;
%y_i(t_count+1)=5;
%z_i(t_count+1)=-15;

%Size of Sensor Footprint
Ri=48;
alpha=(1/2)*120*pi/180;
update_count=0;

%Change on 5/11/16

h=ones(k,1);

%%%%New on 5/19/16
coverror=zeros(1,length(time));
coverror(1)=1;
ui_save=zeros(k,length(time));
vi_save=zeros(k,length(time));
wi_save=zeros(k,length(time));
ri_save=zeros(k,length(time));
si_save=zeros(k,length(time));
lat_save=zeros(k,length(time));
long_save=zeros(k,length(time));
alt_save=zeros(k,length(time));
%figure('units','normalized','outerposition',[0 0 1 1])
t=0;
cap_t=t*ones(k,1);

ui=zeros(k,1);
vi=zeros(k,1);
wi=zeros(k,1);
ri=zeros(k,1);
si=zeros(k,1);
S_i=zeros(length(x_coord),length(y_coord),length(z_coord),k);


x_dest=zeros(1,k);
y_dest=zeros(1,k);
z_dest=zeros(1,k);
x_dest(:)=min(x_coord)-1000*dx;
y_dest(:)=min(y_coord)-1000*dy;
z_dest(:)=min(z_coord)-1000*dz;
eul=zeros(2,3);

while 1  
   
    %tic
    t_count=t_count+1;

   x_i(1,t_count) = x_1*(200/25)+100; % convert to MATLAB coordinates;

    y_i(1,t_count) = y_1*(200/25)+100;

    z_i(1,t_count) = z_1*(100/10)-100;

    lat_save(1,t_count)=lat_1;

    long_save(1,t_count)=long_1;

    alt_save(1,t_count)=alt_1;

    % Convert quaternion parameters to Euler angles and update
    eul(1,:) = quat2eul([qw_1 qx_1 qy_1 qz_1]);

    for kk=1:k
    % Updates coverage code with scaled vicon positions

    % Convert quaternion parameters to Euler angles and update
    

   
    
    
    roll_i(kk,t_count) = eul(kk,3); 
    pitch_i(kk,t_count) =  eul(kk,2); %negative signs transform from 3dr frame
    yaw_i(kk,t_count) =eul(kk,1);
    
    
    
    if x_i(kk,t_count)>max(x_coord)
        x_i(kk,t_count)=max(x_coord);
    end
    if x_i(kk,t_count)<min(x_coord)
        x_i(kk,t_count)=min(x_coord);
    end
    
    if y_i(kk,t_count)>max(y_coord)
        y_i(kk,t_count)=max(y_coord);
    end
    if y_i(kk,t_count)<min(y_coord)
        y_i(kk,t_count)=min(y_coord);
    end
    
    if z_i(kk,t_count)>max(z_coord)
        z_i(kk,t_count)=max(z_coord);
    end
    if z_i(kk,t_count)<min(z_coord)
        z_i(kk,t_count)=min(z_coord);
    end  
        curr_x_Indx_Num(kk)=int64(((1/dx)*(x_i(kk,t_count)+dx)));
        curr_y_Indx_Num(kk)=int64(((1/dy)*(y_i(kk,t_count)+dy)));
        curr_z_Indx_Num(kk)=int64(((1/dz)*(max(abs(z_coord))+(z_i(kk,t_count)+dz))));
x_left=max(1,curr_x_Indx_Num-1*(Ri+2*r_i)/dx);
x_right=min(length(x_coord),curr_x_Indx_Num+1*(Ri+2*r_i)/dx);
y_left=max(1,curr_y_Indx_Num-1*(Ri+2*r_i)/dy);
y_right=min(length(y_coord),curr_y_Indx_Num+1*(Ri+2*r_i)/dy);
z_left=max(1,curr_z_Indx_Num-1*(Ri+2*r_i)/dz);
z_right=min(length(z_coord),curr_z_Indx_Num+1*(Ri+2*r_i)/dz);
    end
    Q_copy=Q;
    Q_mod1=Q;

    %Now save a copy for Global Coverage
    Q_mod_glob_1=Q_mod1;

    %Now encode C* collision Avoidance

    
    
    
    
    
    Q_copy_1=Q_mod1(x_left(1):x_right(1),y_left(1):y_right(1),z_left(1):z_right(1));

   
    
    Si_1=zeros(x_right(1)-x_left(1)+1,y_right(1)-y_left(1)+1,z_right(1)-z_left(1)+1);

    
   for jj=1:k
       if jj==1;
            if Global_Flag(jj)==0;
           
            [command_1,Si_1(:,:,:,jj)]=Local_Cov2(Q_copy_1,[x_i(jj,t_count),y_i(jj,t_count),z_i(jj,t_count),pitch_i(jj,t_count),yaw_i(jj,t_count),roll_i(jj,t_count)],x_coord,y_coord,z_coord,dx,dy,dz,K_u,K_v,K_w,K_r,K_s,Ri,alpha,c_star,r_i);
            
            ui(jj)=command_1(1);
            vi(jj)=command_1(2);
            wi(jj)=command_1(3);
            qi(jj)=command_1(4);
            ri(jj)=command_1(5);
            si(jj)=command_1(6);
            
            else
            [command_1,Si_1(:,:,:,jj),orient_norm(jj)]=Global_Cov([x_i(1,t_count),y_i(1,t_count),z_i(1,t_count),pitch_i(1,t_count),yaw_i(1,t_count),roll_i(1,t_count)],[x_i(2,t_count),y_i(2,t_count),z_i(2,t_count),pitch_i(2,t_count),yaw_i(2,t_count),roll_i(2,t_count)],[2*max(x_coord),2*max(y_coord),2*min(z_coord),0,0],x_dest(jj),y_dest(jj),z_dest(jj),pitch_dest(jj),yaw_dest(jj),x_coord,y_coord,z_coord,dx,dy,dz,Ri,alpha,r_i);
            % Use these commands for quadrotor control
            ui(jj)=command_1(1);
            vi(jj)=command_1(2);
            wi(jj)=command_1(3);
            qi(jj)=command_1(4);
            ri(jj)=command_1(5);
            si(jj)=command_1(6);  
                
            end
       end
       
       if jj==2;
            if Global_Flag(jj)==0;
            [command_2,Si_2(:,:,:,jj)]=Local_Cov2(Q_copy_2,[x_i(2,t_count),y_i(2,t_count),z_i(2,t_count),pitch_i(2,t_count),yaw_i(2,t_count),roll_i(2,t_count)],x_coord,y_coord,z_coord,dx,dy,dz,K_u,K_v,K_w,K_r,K_s,Ri,alpha,c_star,r_i);
            ui(jj)=command_2(1);
            vi(jj)=command_2(2);
            wi(jj)=command_2(3);
            qi(jj)=command_2(4);
            ri(jj)=command_2(5);
            si(jj)=command_2(6);
            
            else
            [command_2,Si_2(:,:,:,jj),orient_norm(jj)]=Global_Cov([x_i(2,t_count),y_i(2,t_count),z_i(2,t_count),pitch_i(2,t_count),yaw_i(2,t_count),roll_i(2,t_count)],[x_i(1,t_count),y_i(1,t_count),z_i(1,t_count),pitch_i(1,t_count),yaw_i(1,t_count),roll_i(1,t_count)],[2*max(x_coord),2*max(y_coord),2*min(z_coord),0,0],x_dest(jj),y_dest(jj),z_dest(jj),pitch_dest(jj),yaw_dest(jj),x_coord,y_coord,z_coord,dx,dy,dz,Ri,alpha,r_i);
            ui(jj)=command_2(1);
            vi(jj)=command_2(2);
            wi(jj)=command_2(3);
            qi(jj)=command_2(4);
            ri(jj)=command_2(5);
            si(jj)=command_2(6);    
                
            end
        end
       

   end

%---------------------------- END ------------------------------------%    
    
%%%%Okay Now only update the relevant parts of the coverage Map  

Q(x_left(1):x_right(1),y_left(1):y_right(1),z_left(1):z_right(1))=Q(x_left(1):x_right(1),y_left(1):y_right(1),z_left(1):z_right(1))+Si_1*dt;

e_hat_(1)=trapz(trapz(trapz((max(0,c_star-Q(x_left(1):x_right(1),y_left(1):y_right(1),z_left(1):z_right(1))).^3).*Si_1)))*dx*dy*dz;

%%%%%%New Code from 5/19/16 Implement a switched law here for a Proportional controller
%for waypoint redirection. NOTE THAT qi,ri,si have changed throughout the
%code. I Have corrected them to be the same as in the paper. Also look
%specifically at the rolldot on line 461. Now includes qi

for kk=1:k

%Send Message for agent k.
%------------------------ Updates ctrl_msg ---------------------------%


%Begin Code to Check if on boundary and command fly back inside

Rot_Mat=[cos(pitch_i(kk,t_count))*cos(yaw_i(kk,t_count)),(sin(roll_i(kk,t_count))*sin(pitch_i(kk,t_count))*cos(yaw_i(kk,t_count))-cos(roll_i(kk,t_count))*sin(yaw_i(kk,t_count))),(cos(roll_i(kk,t_count))*sin(pitch_i(kk,t_count))*cos(yaw_i(kk,t_count))+sin(roll_i(kk,t_count))*sin(yaw_i(kk,t_count))); ...
         cos(pitch_i(kk,t_count))*sin(yaw_i(kk,t_count)),(sin(roll_i(kk,t_count))*sin(pitch_i(kk,t_count))*sin(yaw_i(kk,t_count))+cos(roll_i(kk,t_count))*cos(yaw_i(kk,t_count))),(cos(roll_i(kk,t_count))*sin(pitch_i(kk,t_count))*sin(yaw_i(kk,t_count))-sin(roll_i(kk,t_count))*cos(yaw_i(kk,t_count))); ...
         -sin(pitch_i(kk,t_count)),(sin(roll_i(kk,t_count))*cos(pitch_i(kk,t_count))),(cos(roll_i(kk,t_count))*cos(pitch_i(kk,t_count)))];

bound_vx=[0;0;0];
bound_vy=[0;0;0];
bound_vz=[0;0;0];
bound_vc=[0;0;0];
bound_flag=0;
    if x_i(kk,t_count)==max(x_coord)
        bound_vx=inv(Rot_Mat)*[-U_max;0;0];
        bound_flag=1;
    end
    if x_i(kk,t_count)==min(x_coord)
        bound_vx=inv(Rot_Mat)*[U_max;0;0];
        bound_flag=1;
    end
    
    if y_i(kk,t_count)==max(y_coord)
        bound_vy=inv(Rot_Mat)*[0;-U_max;0]; 
        bound_flag=1;
    end
    if y_i(kk,t_count)==min(y_coord)
        bound_vy=inv(Rot_Mat)*[0;U_max;0]; 
        bound_flag=1;
    end
    
    if z_i(kk,t_count)==max(z_coord)
        bound_vz=inv(Rot_Mat)*[0;0;-U_max]; 
        bound_flag=1;
    end
    if z_i(kk,t_count)==min(z_coord)
        bound_vz=inv(Rot_Mat)*[0;0;U_max]; 
        bound_flag=1;
    end
   if bound_flag==1
       bound_vc=bound_vx+bound_vy+bound_vz;
       ui(kk)=bound_vc(1);
       vi(kk)=bound_vc(2);
       wi(kk)=bound_vc(3);
   end
%End Code to Check if on boundary and command fly back inside
%if kk==2
%[ui(2) vi(2)]
%e


%TEMPORARY PLEASE REMOVE THIS LATER WITH INTELLIGENT GAINS!!!!
%%%%%%%%%%%%
if ui(kk)>.5
    ui=.5;
end
if ui(kk)<-.5
    ui=-.5;
end
  if vi(kk)>.5
    vi=.5;
end
if vi(kk)<-.5
    vi=-.5;
end
if wi(kk)>.5
    wi=.5;
end
if wi(kk)<-.5
    wi=-.5;
end
%%%%%%%%%%%

ctrl_msg(1).Twist.Linear.X = ui(kk);
ctrl_msg(1).Twist.Linear.Y = vi(kk);
ctrl_msg(1).Twist.Linear.Z = wi(kk);
ctrl_msg(1).Twist.Angular.Z = si(kk);


%Will's Moving Average Filter
%if t_count>10
%ctrl_msg(kk).X=mean([ui(kk),ui_save(kk,h(kk)-1),ui_save(kk,h(kk)-2),ui_save(kk,h(kk)-3),ui_save(kk,h(kk)-4)]);
%ctrl_msg(kk).Y=mean([vi(kk),vi_save(kk,h(kk)-1),vi_save(kk,h(kk)-2),vi_save(kk,h(kk)-3),vi_save(kk,h(kk)-4)]);
%ctrl_msg(kk).Z=mean([wi(kk),wi_save(kk,h(kk)-1),wi_save(kk,h(kk)-2),wi_save(kk,h(kk)-3),wi_save(kk,h(kk)-4)]);
%ctrl_msg(kk).Yaw=mean([si(kk),si_save(kk,h(kk)-1),si_save(kk,h(kk)-2),si_save(kk,h(kk)-3),si_save(kk,h(kk)-4)]);
%end

%Or Unfiltered by Alternative
%ctrl_msg(kk).X=ui(kk);
%ctrl_msg(kk).Y=vi(kk);
%ctrl_msg(kk).Z=wi(kk);
%ctrl_msg(kk).Yaw=si(kk);


%------------------------- Simulation model update--------------------%

% %New term on Rolldot. Also I have fixed the ri and si names throughout the code as they were inconsistent with the paper 5/19/16 
% rolldot=qi(kk)+sin(roll_i(kk,t_count))*tan(pitch_i(kk,t_count))*ri(kk)+cos(roll_i(kk,t_count))*tan(pitch_i(kk,t_count))*si(kk);
% pitchdot=cos(roll_i(kk,t_count))*ri(kk)-sin(roll_i(kk,t_count))*si(kk);
% yawdot=sin(roll_i(kk,t_count))*sec(pitch_i(kk,t_count))*ri(kk)+cos(roll_i(kk,t_count))*sec(pitch_i(kk,t_count))*si(kk);

% Propagated states from the simulation
x_i(kk,t_count+1) = x_i(kk,t_count);
y_i(kk,t_count+1) = y_i(kk,t_count);
z_i(kk,t_count+1) = z_i(kk,t_count);



roll_i(kk,t_count+1) = roll_i(kk,t_count);
pitch_i(kk,t_count+1) = pitch_i(kk,t_count);
yaw_i(kk,t_count+1) = yaw_i(kk,t_count);

%%%%%New Code from 5/19/16
ui_save(kk,h(kk))=ui(kk);
vi_save(kk,h(kk))=vi(kk);
wi_save(kk,h(kk))=wi(kk);
ri_save(kk,h(kk))=ri(kk);
si_save(kk,h(kk))=si(kk);

if Global_Flag(kk)==1 && sqrt((x_i(kk,t_count+1)-x_dest(kk))^2+(y_i(kk,t_count+1)-y_dest(kk))^2+(z_i(kk,t_count+1)-z_dest(kk))^2)<cap_tol && orient_norm(kk)<orient_tol;
    Global_Flag(kk)=0;
    fprintf('Captured new Orientation at waypoint for Agent:\n')
    x_dest(kk)=min(x_coord)-1000*dx;
    y_dest(kk)=min(y_coord)-1000*dy;
    z_dest(kk)=min(z_coord)-1000*dz;
    cap_t(kk)=t;
    j_tilde(kk)=0;

end

if h(kk)==1
    coverror_init=trapz(trapz(trapz(max(0,c_star-Q(:,:,:)).^3)))*dx*dy*dz;
end
 
coverror(h(kk))=trapz(trapz(trapz(max(0,c_star-Q(:,:,:)).^3)))*dx*dy*dz;
coverror(h(kk))=coverror(h(kk))/coverror_init;
    

  h(kk)=h(kk)+1;
  %Only to make while loop work. This gets overwritten each cycle. 
  coverror(h(kk))=coverror(h(kk)-1); 
    
%Global Coverage Code!!! All new here 5/19/16 basically until end of code
%if t>(1+cap_t(kk))
%if abs(coverror(h(kk)-10)-coverror(h(kk)-1))<Ep_star && Global_Flag(kk)==0
% NEW CODE 7/12/16. Let's use a velocity command threshold instead of a
% coverage threshold. That way global coverage can occur for individual
% agents not just all of them at once


     if exist('j_tilde','var')==1   
     if max(size(j_tilde))==2
    if sum(j_tilde==[0,0])==2
        clear j_tilde;
    end
     end
     end
     if exist('j_tilde','var')==1
         if max(size(j_tilde))==1
    if sum(j_tilde==[0])==1
        clear j_tilde;
    end
        end
     end

    
     e_hat_save(:,t_count)=e_hat_;
if  t_count>101
if mean(e_hat_save(kk,t_count-100:t_count))<Ep_star && Global_Flag(kk)==0 && 1==0 %disabled duh
fprintf('Entering Global Coverage Mode\n')    
%Publish Zero incase Global Lags
ctrl_msg(1).X=0;
ctrl_msg(1).Y=0;
ctrl_msg(1).Z=0;
ctrl_msg(1).Yaw=0;
send(ctrl_pub(1),ctrl_msg(1));  
ctrl_msg(2).X=0;
ctrl_msg(2).Y=0;
ctrl_msg(2).Z=0;
ctrl_msg(2).Yaw=0;
send(ctrl_pub(2),ctrl_msg(2));  

x_glo_dex(kk)=1;
x_glo_cen(kk,x_glo_dex(kk))=floor((1/2)*sqrt(2)*Ri/dx);
y_glo_dex(kk)=1;
y_glo_cen(kk,y_glo_dex(kk))=floor((1/2)*sqrt(2)*Ri/dy);
z_glo_dex(kk)=1;
z_glo_cen(kk,z_glo_dex(kk))=floor((1/2)*sqrt(2)*Ri/dz);
glo_dex(kk)=1;
tic
while x_glo_dex(kk)<length(Q(1,:,:))
    y_glo_dex(kk)=1;
    while y_glo_dex(kk)<length(Q(:,1,:)) 
        z_glo_dex(kk)=1;
        while z_glo_dex(kk)<length(Q(:,:,1))
          
            x_glo_left(kk)=max(1,x_glo_cen(kk,x_glo_dex(kk))-floor((1/2)*sqrt(2)*Ri/dx));
            x_glo_right(kk)=min(length(Q(:,1,1)),x_glo_cen(kk,x_glo_dex(kk))+floor((1/2)*sqrt(2)*Ri/dx));
            y_glo_left(kk)=max(1,y_glo_cen(kk,y_glo_dex(kk))-floor((1/2)*sqrt(2)*Ri/dy));
            y_glo_right(kk)=min(length(Q(1,:,1)),y_glo_cen(kk,y_glo_dex(kk))+floor((1/2)*sqrt(2)*Ri/dy));
            z_glo_left(kk)=max(1,z_glo_cen(kk,z_glo_dex(kk))-floor((1/2)*sqrt(2)*Ri/dz));
            z_glo_right(kk)=min(length(Q(1,1,:)),z_glo_cen(kk,z_glo_dex(kk))+floor((1/2)*sqrt(2)*Ri/dz));
            %Calculate the C_hat expression. Coverage in each subdomain
            %NEW CODE CHANGE ON 7/12/16
 %I'm going to remove a *dx*dy*dz from the insie of the dirac
            %functions
            if kk==1
            if abs(c_star*(x_glo_right(kk)-x_glo_left(kk))*(y_glo_right(kk)-y_glo_left(kk))*(z_glo_right(kk)-z_glo_left(kk))-trapz(trapz(trapz(min(c_star,Q_mod_glob_1(x_glo_left(kk):x_glo_right(kk),y_glo_left(kk):y_glo_right(kk),z_glo_left(kk):z_glo_right(kk))),3).*dz,2).*dy,1).*dx)<10^-10
            dirac_sub=inf;
            else
            dirac_sub=0;
            end
            C_hat(x_glo_dex(kk),y_glo_dex(kk),z_glo_dex(kk),kk)= trapz(trapz(trapz(min(c_star,Q_mod_glob_1(x_glo_left(kk):x_glo_right(kk),y_glo_left(kk):y_glo_right(kk),z_glo_left(kk):z_glo_right(kk))),3).*dz,2).*dy,1).*dx ...
                                                  +dirac_sub;
            if (sqrt((x_coord(x_glo_cen(1,x_glo_dex(1)))-x_dest(2))^2+(y_coord(y_glo_cen(1,y_glo_dex(1)))-y_dest(2))^2)^2)==0
                
            C_hat(x_glo_dex(kk),y_glo_dex(kk),z_glo_dex(kk),kk)=inf;
            end
                                              
            end
            if kk==2
            if abs(c_star*(x_glo_right(kk)-x_glo_left(kk))*(y_glo_right(kk)-y_glo_left(kk))*(z_glo_right(kk)-z_glo_left(kk))-trapz(trapz(trapz(min(c_star,Q_mod_glob_2(x_glo_left(kk):x_glo_right(kk),y_glo_left(kk):y_glo_right(kk),z_glo_left(kk):z_glo_right(kk))),3).*dz,2).*dy,1).*dx)<10^-10
            dirac_sub=inf;
            else
            dirac_sub=0;
            end
            C_hat(x_glo_dex(kk),y_glo_dex(kk),z_glo_dex(kk),kk)= trapz(trapz(trapz(min(c_star,Q_mod_glob_2(x_glo_left(kk):x_glo_right(kk),y_glo_left(kk):y_glo_right(kk),z_glo_left(kk):z_glo_right(kk))),3).*dz,2).*dy,1).*dx ...
                                                  +dirac_sub;
            if (sqrt((x_coord(x_glo_cen(2,x_glo_dex(2)))-x_dest(1))^2+(y_coord(y_glo_cen(2,y_glo_dex(2)))-y_dest(1))^2)^2)==0
                
            C_hat(x_glo_dex(kk),y_glo_dex(kk),z_glo_dex(kk),kk)=inf;
            
            end
            end
            if kk==3
            if abs(c_star*(x_glo_right(kk)-x_glo_left(kk))*(y_glo_right(kk)-y_glo_left(kk))*(z_glo_right(kk)-z_glo_left(kk))-trapz(trapz(trapz(min(c_star,Q_mod_glob_3(x_glo_left(kk):x_glo_right(kk),y_glo_left(kk):y_glo_right(kk),z_glo_left(kk):z_glo_right(kk))),3).*dz,2).*dy,1).*dx)<10^-10
            dirac_sub=inf;
            else
            dirac_sub=0;
            end
                C_hat(x_glo_dex(kk),y_glo_dex(kk),z_glo_dex(kk),kk)= trapz(trapz(trapz(min(c_star,Q_mod_glob_3(x_glo_left(kk):x_glo_right(kk),y_glo_left(kk):y_glo_right(kk),z_glo_left(kk):z_glo_right(kk))),3).*dz,2).*dy,1).*dx ...
                                                  +dirac_sub;  
            if (sqrt((x_coord(x_glo_cen(3,x_glo_dex(3)))-x_dest(1))^2+(y_coord(y_glo_cen(3,y_glo_dex(3)))-y_dest(1))^2+(z_coord(z_glo_cen(3,z_glo_dex(3)))-z_dest(1))^2)<2*Ri) || (sqrt((x_coord(x_glo_cen(3,x_glo_dex(3)))-x_dest(2))^2+(y_coord(y_glo_cen(3,y_glo_dex(3)))-y_dest(2))^2+(z_coord(z_glo_cen(3,z_glo_dex(3)))-z_dest(2))^2)<2*Ri)
                
            C_hat(x_glo_dex(kk),y_glo_dex(kk),z_glo_dex(kk),kk)=inf;
            end
                                              
            end
            %Rearrange information into a more logical matrix here
            C_hat_concat(1,glo_dex(kk),kk)=C_hat(x_glo_dex(kk),y_glo_dex(kk),z_glo_dex(kk),kk);
            C_hat_concat(2,glo_dex(kk),kk)=x_glo_cen(kk,x_glo_dex(kk));
            C_hat_concat(3,glo_dex(kk),kk)=y_glo_cen(kk,y_glo_dex(kk));
            C_hat_concat(4,glo_dex(kk),kk)=z_glo_cen(kk,z_glo_dex(kk));
         
               if exist('j_tilde')==1

                   if squeeze(max((C_hat_concat(2,nonzeros(j_tilde(:)),:)==C_hat_concat(2,glo_dex(kk),kk)) +  (C_hat_concat(3,nonzeros(j_tilde(:)),:)==C_hat_concat(3,glo_dex(kk),kk)) +  (C_hat_concat(4,nonzeros(j_tilde(:)),:)==C_hat_concat(4,glo_dex(kk),kk))))==[3;3]
                C_hat_concat(1,glo_dex(kk),kk)=inf;
                    end
                    end
               
           
            glo_dex(kk)=glo_dex(kk)+1;
            if z_glo_right(kk)<length(Q(1,1,:))
            z_glo_cen(kk,z_glo_dex(kk)+1)=min(length(Q(1,1,:))-floor((1/2)*sqrt(2)*Ri/dz),z_glo_cen(kk,z_glo_dex(kk))+2*floor((1/2)*sqrt(2)*Ri/dz));
            z_glo_dex(kk)=z_glo_dex(kk)+1;
            else
                break
            end
        end
        if y_glo_right(kk)<length(Q(1,:,1))
            y_glo_cen(kk,y_glo_dex(kk)+1)=min(length(Q(1,:,1))-floor((1/2)*sqrt(2)*Ri/dy),y_glo_cen(kk,y_glo_dex(kk))+2*floor((1/2)*sqrt(2)*Ri/dy));
            y_glo_dex(kk)=y_glo_dex(kk)+1;
            else
                break
        end
    end
    
        if x_glo_right(kk)<length(Q(:,1,1))
            x_glo_cen(kk,x_glo_dex(kk)+1)=min(length(Q(:,1,1))-floor((1/2)*sqrt(2)*Ri/dx),x_glo_cen(kk,x_glo_dex(kk))+2*floor((1/2)*sqrt(2)*Ri/dx));
            x_glo_dex(kk)=x_glo_dex(kk)+1;
            else
                break
        end
end
toc
big_gam(kk)=trapz(lil_gam_1*abs(ui_save(kk,:))+lil_gam_2*abs(vi_save(kk,:))+lil_gam_3*abs(wi_save(kk,:))+lil_gam_4*abs(ri_save(kk,:))+lil_gam_5*abs(si_save(kk,:)))*dt;



[trash(kk),j_tilde(kk)]=min((Lam_0+big_gam(kk))*sqrt((repmat(x_i(kk,t_count+1),1,length(C_hat_concat(2,:,kk)))-x_coord(C_hat_concat(2,:,kk))).^2+(repmat(y_i(kk,t_count+1),1,length(C_hat_concat(3,:,kk)))-y_coord(C_hat_concat(3,:,kk))).^2+(repmat(z_i(kk,t_count+1),1,length(C_hat_concat(4,:,kk)))-zee_coord(C_hat_concat(4,:,kk))).^2)+Lam_1*C_hat_concat(1,:,kk));
x_dest(kk)=x_coord(C_hat_concat(2,j_tilde(kk),kk));
y_dest(kk)=y_coord(C_hat_concat(3,j_tilde(kk),kk));
z_dest(kk)=z_coord(C_hat_concat(4,j_tilde(kk),kk));
orient_glo_dex(kk)=1;

for yaw_test=0:alpha:2*pi %Need to avoid gimbal lock
     for pitch_test=0

        curr_x_Indx_Num(kk)=int64(((1/dx)*(x_dest(kk)+dx)));
        curr_y_Indx_Num(kk)=int64(((1/dy)*(y_dest(kk)+dy)));
        curr_z_Indx_Num(kk)=int64(((1/dz)*(max(abs(z_coord))+(z_dest(kk)+dz))));

x_left(kk)=max(1,curr_x_Indx_Num(kk)-1.5*Ri/dx);
x_right(kk)=min(length(x_coord),curr_x_Indx_Num(kk)+1.5*Ri/dx);
y_left(kk)=max(1,curr_y_Indx_Num(kk)-1.5*Ri/dy);
y_right(kk)=min(length(y_coord),curr_y_Indx_Num(kk)+1.5*Ri/dy);
z_left(kk)=max(1,curr_z_Indx_Num(kk)-1.5*Ri/dz);
z_right(kk)=min(length(z_coord),curr_z_Indx_Num(kk)+1.5*Ri/dz);

x_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk) = repmat(x_coord(x_left(kk):x_right(kk))',1,length(y_coord(y_left(kk):y_right(kk))),length(z_coord(z_left(kk):z_right(kk))));
y_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk) = repmat(y_coord(y_left(kk):y_right(kk)),length(x_coord(x_left(kk):x_right(kk))),1,length(z_coord(z_left(kk):z_right(kk))));
z_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk) = repmat(z_coord(z_left(kk):z_right(kk)),length(x_coord(x_left(kk):x_right(kk))),length(y_coord(y_left(kk):y_right(kk))),1);
phi(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))...
    =acos((((x_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)-x_dest(kk)).*cos(yaw_test).*cos(pitch_test)...
    +(y_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)-y_dest(kk)).*sin(yaw_test).*cos(pitch_test)-...
    (z_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)-z_dest(kk)).*sin(pitch_test))...
    ./sqrt((x_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)-x_dest(kk)).^2....
    +(y_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)-y_dest(kk)).^2....
    +(z_mat(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)-z_dest(kk)).^2)));

c1(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))=Ri^2-(repmat(x_coord(x_left(kk):x_right(kk))',1,length(y_coord(y_left(kk):y_right(kk))),length(z_coord(z_left(kk):z_right(kk))))-x_dest(kk)).^2-(repmat(y_coord(y_left(kk):y_right(kk)),length(x_coord(x_left(kk):x_right(kk))),1,length(z_coord(z_left(kk):z_right(kk))))-y_dest(kk)).^2-(repmat(z_coord(z_left(kk):z_right(kk)),length(x_coord(x_left(kk):x_right(kk))),length(y_coord(y_left(kk):y_right(kk))),1)-z_dest(kk)).^2;
c2(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))=alpha-phi(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk));
c3(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))=alpha+phi(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk));
B(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))=(1/max(0,c1(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))))+(1/max(0,c2(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))))+(1/max(0,c3(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))));
Si(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)=1./B(kk,x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk));
Si(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)=Si(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk)./max(max(max(Si(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk))));

Arg_Omega(kk,1,orient_glo_dex(kk))=trapz(trapz(trapz((3*(c_star-Q(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk))).^2).*Si(x_left(kk):x_right(kk),y_left(kk):y_right(kk),z_left(kk):z_right(kk),kk),3).*dz,2).*dy,1).*dx;
Arg_Omega(kk,2,orient_glo_dex(kk))=yaw_test;
Arg_Omega(kk,3,orient_glo_dex(kk))=pitch_test;
    
orient_glo_dex(kk)=orient_glo_dex(kk)+1;

     end
end
[trash(kk),Arg_dex(kk)]=max(Arg_Omega(kk,1,:));
        yaw_dest(kk)=Arg_Omega(kk,2,Arg_dex(kk));
        if yaw_dest(kk)>pi;
            yaw_dest(kk)=yaw_dest(kk)-2*pi;
        end
pitch_dest(kk)=0;

%%%%%%%%%%%%%%%%%%%%%%%%%
%Test Values
%x_dest=20;
%y_dest=20;
%z_dest=-10;
%pitch_dest=pi/4;
%yaw_dest=pi;

Global_Flag(kk)=1;
    end
end
end


act_accum_1(t_count)=abs(ui(1))+abs(vi(1))+abs(wi(1))+abs(ri(1))+abs(si(1));



%[trash,dest2]=max(e(:));
%clear trash

% end
%end
%end
%  PQ(:,:,:)=(((Q-Q_copy)*dt));
% % 
%  test_count_1=0;
%  clear Test_shape_1
%  
%  for xx=1:1:length(x_coord)
%          for yy=1:1:length(y_coord)
%              for zz=1:1:length(z_coord)
% % %%%%%DELETE BELOW AND THEN GO UP AND DELETE test_count initialization
%  if PQ(xx,yy,zz)>0
%  test_count_1=test_count_1+1;
%  Test_shape_1(test_count_1,:)=[x_coord(xx) y_coord(yy) z_coord(zz) PQ(xx,yy,zz)];
%  end
%              end
%          end
%  end
%  
%  %subplot(1,2,2)
%  scatter3(Test_shape_1(:,1),Test_shape_1(:,2),Test_shape_1(:,3),5,Test_shape_1(:,4));
%  hold on;
%  axis([min(x_coord),max(x_coord),min(y_coord),max(y_coord),min(z_coord),max(z_coord)])
%  
%  coverror=trapz(trapz(trapz(max(0,c_star-Q(:,:,:)).^3)))*dx*dy*dz;
%     %title(['Time = ', num2str((h-1)*dt), ' sec. Coverage Error= ',num2str(coverror)])
%  
%  %title(['Cov. Time = ', num2str((h(kk)-2)*dt), ' sec. Coverage Error= ',num2str(coverror(h(kk)-1))])
%  %title(['Cov. Time = ', num2str((h(kk)-2)*dt), ' sec. Coverage Error= ',num2str(coverror(1))])
%   
%  plot3([x_i(1,t_count) x_i(1,t_count)],[y_i(1,t_count) y_i(1,t_count)],[min(z_coord) max(z_coord)],'b')
%  
%  hold off;
%  
%  drawnow update
% mov= getframe(gcf);
%     
%     writeVideo(vidObj,mov)
%  
%      clear mov
%     mov = struct('cdata', [],...
%                        'colormap', []);

t=t+dt;

coverr=trapz(trapz(trapz(max(0,c_star-Q(:,:,:)).^3)))*dx*dy*dz;
coverr=coverr/coverror_init;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pos_data = [1 coverr; x_i(kk,t_count+1) y_i(kk,t_count+1); z_i(kk,t_count+1) si(kk)];
% save position.txt -ascii pos_data
% 
% pos_data = importdata('cpp_pos.txt');
% pos_size = size(pos_data);
% 
% while pos_size(1)== 0
%     pos_data = importdata('cpp_pos.txt');
%     pos_size = size(pos_data);
% end
% 
% for kk=1:1
%     
%         if(pos_data(kk,1)<0)
%         pos_data(kk,1) = 0;
%     else if(pos_data(kk,1)>200)
%         pos_data(kk,1) = 200;
%         end
%     end
%     
%     if(pos_data(kk,2)<0)
%         pos_data(kk,2) = 0;
%     else if(pos_data(kk,2)>200)
%         pos_data(kk,2) = 200;
%         end
%     end
%     
%     if(pos_data(kk,3)>0)
%         pos_data(kk,1) = 0;
%     else if(pos_data(kk,3)<-100)
%         pos_data(kk,3) = -100;
%         end
%     end
%     
%     x_i(kk,t_count+1)=pos_data(kk,1);
%     y_i(kk,t_count+1)=pos_data(kk,2);
%     z_i(kk,t_count+1)=pos_data(kk,3);
%     
% end
% 
% fid = fopen('cpp_pos.txt', 'wt');
% fclose(fid);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%[Global_Flag(1) Global_Flag(2)]
Current=[x_i(1,t_count),y_i(1,t_count),z_i(1,t_count) yaw_i(1,t_count) ui(1) vi(1)]

%[volt_1,volt_2]
%ui
%toc
drawnow
    
    
    
    
    
    
    
    
    
    
 
   
   send(ctrl_pub(1),ctrl_msg(1));
   
   waitfor(rate); 
    
end


rosshutdown;