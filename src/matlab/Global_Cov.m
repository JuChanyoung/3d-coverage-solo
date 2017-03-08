%%%%NOTE!!! This code is assuming the vicon scaling of which is twice as
%%%%compressed as xy. So I will go in and change the z-z_friend to have
%%%%multiples of two in it to be a little less conservative in high. That
%%%%way they are actually able to fly over top of eachother at all.


function [command,S_i,orient_norm]=Global_Cov(state_me,state_friend_1,state_friend_2,x_dest,y_dest,z_dest,pitch_dest,yaw_dest,x_coord,y_coord,z_coord,dx,dy,dz,Ri,alpha,r_i)


x_i=state_me(1);
y_i=state_me(2);
z_i=state_me(3);
pitch_i=state_me(4);
yaw_i=state_me(5);
roll_i=state_me(6);




%Old Janky Collision Avoidance
%if sqrt((state_friend_1(1)-state_me(1))^2+(state_friend_1(2)-state_me(2))^2+(state_friend_1(3)-state_me(3))^2)<Ri
%    Sub_com_vx_1=(state_me(1)-state_friend_1(1))/sqrt((state_friend_1(1)-state_me(1))^2+(state_friend_1(2)-state_me(2))^2+(state_friend_1(3)-state_me(3))^2);
%    Sub_com_vy_1=(state_me(2)-state_friend_1(2))/sqrt((state_friend_1(1)-state_me(1))^2+(state_friend_1(2)-state_me(2))^2+(state_friend_1(3)-state_me(3))^2);
%    Sub_com_vz_1=(state_me(3)-state_friend_1(3))/sqrt((state_friend_1(1)-state_me(1))^2+(state_friend_1(2)-state_me(2))^2+(state_friend_1(3)-state_me(3))^2);
%    command(1)=Sub_com_vx_1;
%    command(2)=Sub_com_vy_1;
%    command(3)=Sub_com_vz_1;
%    command(4)=0;
%    command(5)=0;
%    command(6)=0;
%end
%if sqrt((state_friend_2(1)-state_me(1))^2+(state_friend_2(2)-state_me(2))^2+(state_friend_2(3)-state_me(3))^2)<Ri
%    Sub_com_vx_2=(state_me(1)-state_friend_2(1))/sqrt((state_friend_2(1)-state_me(1))^2+(state_friend_2(2)-state_me(2))^2+(state_friend_2(3)-state_me(3))^2);
%    Sub_com_vy_2=(state_me(2)-state_friend_2(2))/sqrt((state_friend_2(1)-state_me(1))^2+(state_friend_2(2)-state_me(2))^2+(state_friend_2(3)-state_me(3))^2);
%    Sub_com_vz_2=(state_me(3)-state_friend_2(3))/sqrt((state_friend_2(1)-state_me(1))^2+(state_friend_2(2)-state_me(2))^2+(state_friend_2(3)-state_me(3))^2);
%    command(1)=Sub_com_vx_2;
%    command(2)=Sub_com_vy_2;
%    command(3)=Sub_com_vz_2;
%    command(4)=0;
%    command(5)=0;
%    command(6)=0;
%end
%if sqrt((state_friend_1(1)-state_me(1))^2+(state_friend_1(2)-state_me(2))^2+(state_friend_1(3)-state_me(3))^2)<Ri && sqrt((state_friend_2(1)-state_me(1))^2+(state_friend_2(2)-state_me(2))^2+(state_friend_2(3)-state_me(3))^2)<Ri
 %   command(1)=Sub_com_vx_1+Sub_com_vx_2;
  %  command(2)=Sub_com_vy_1+Sub_com_vy_2;
   % command(3)=Sub_com_vz_1+Sub_com_vz_2;
    %command(4)=0;
 %   command(5)=0;
  %  command(6)=0;
%end

%%%%%Global Collision Avoidance 8/12/16
%Note that I am making the dx dy dz smaller in this section. In the old
%version, while doing +dz computations the agents would have a distance
%closer together tahn that of d_s. THus creating imaginary numbers and
%blowing everything to shit.
%Nominal
dx=dx/100;
dy=dy/100;
dz=dz/100;
d_s=2*r_i;
R_z=(Ri+2*r_i)/2;
V_i0=(x_i-x_dest)^2+(y_i-y_dest)^2+(2*(z_i-z_dest))^2;
c_i1=(x_i-state_friend_1(1))^2+(y_i-state_friend_1(2))^2+(2*(z_i-state_friend_1(3)))^2-d_s^2;
c_i2=(x_i-state_friend_2(1))^2+(y_i-state_friend_2(2))^2+(2*(z_i-state_friend_2(3)))^2-d_s^2;
b_i1=-100*log(c_i1);
b_i2=-100*log(c_i2);
q_i1=b_i1;
q_i2=b_i2;
V_p_i1=q_i1^2;
V_p_i2=q_i2^2;
A=-2/(R_z-Ri)^3;
B=3*(R_z+Ri)/(R_z-Ri)^3;
C=-6*R_z*Ri/(R_z-Ri)^3;
D=Ri^2*(3*R_z-Ri)/(R_z-Ri)^3;
di1=sqrt((x_i-state_friend_1(1))^2+(y_i-state_friend_1(2))^2+(2*(z_i-state_friend_1(3)))^2);
di2=sqrt((x_i-state_friend_2(1))^2+(y_i-state_friend_2(2))^2+(2*(z_i-state_friend_2(3)))^2);
%Define The sigmas
%Sig 1
if di1>=Ri
    sigi1=0;
elseif di1>R_z && di1<Ri
    sigi1=A*di1^3+B*di1^2+C*di1+D;
else
    sigi1=1;
end
%Sig 2
if di2>=Ri
    sigi2=0;
elseif di2>R_z && di2<Ri
    sigi2=A*di2^3+B*di2^2+C*di2+D;
else
    sigi2=1;
end
V_i1=sigi1*V_p_i1;
V_i2=sigi2*V_p_i2;
v_i=V_i0+V_i1+V_i2;
V_i=v_i/(1+v_i);
%Plus dx
x_i=x_i+dx;
d_s=2*r_i;
R_z=(Ri+2*r_i)/2;
V_i0=(x_i-x_dest)^2+(y_i-y_dest)^2+(2*(z_i-z_dest))^2;
c_i1=(x_i-state_friend_1(1))^2+(y_i-state_friend_1(2))^2+(2*(z_i-state_friend_1(3)))^2-d_s^2;
c_i2=(x_i-state_friend_2(1))^2+(y_i-state_friend_2(2))^2+(2*(z_i-state_friend_2(3)))^2-d_s^2;
b_i1=-100*log(c_i1);
b_i2=-100*log(c_i2);
q_i1=b_i1;
q_i2=b_i2;
V_p_i1=q_i1^2;
V_p_i2=q_i2^2;
A=-2/(R_z-Ri)^3;
B=3*(R_z+Ri)/(R_z-Ri)^3;
C=-6*R_z*Ri/(R_z-Ri)^3;
D=Ri^2*(3*R_z-Ri)/(R_z-Ri)^3;
di1=sqrt((x_i-state_friend_1(1))^2+(y_i-state_friend_1(2))^2+(2*(z_i-state_friend_1(3)))^2);
di2=sqrt((x_i-state_friend_2(1))^2+(y_i-state_friend_2(2))^2+(2*(z_i-state_friend_2(3)))^2);
%Define The sigmas
%Sig 1
if di1>=Ri
    sigi1=0;
elseif di1>R_z && di1<Ri
    sigi1=A*di1^3+B*di1^2+C*di1+D;
else
    sigi1=1;
end
%Sig 2
if di2>=Ri
    sigi2=0;
elseif di2>R_z && di2<Ri
    sigi2=A*di2^3+B*di2^2+C*di2+D;
else
    sigi2=1;
end
V_i1=sigi1*V_p_i1;
V_i2=sigi2*V_p_i2;
v_i=V_i0+V_i1+V_i2;
V_i_pdx=v_i/(1+v_i);
x_i=x_i-dx;
%minus dx
x_i=x_i-dx;
d_s=2*r_i;
R_z=(Ri+2*r_i)/2;
V_i0=(x_i-x_dest)^2+(y_i-y_dest)^2+(2*(z_i-z_dest))^2;
c_i1=(x_i-state_friend_1(1))^2+(y_i-state_friend_1(2))^2+(2*(z_i-state_friend_1(3)))^2-d_s^2;
c_i2=(x_i-state_friend_2(1))^2+(y_i-state_friend_2(2))^2+(2*(z_i-state_friend_2(3)))^2-d_s^2;
b_i1=-100*log(c_i1);
b_i2=-100*log(c_i2);
q_i1=b_i1;
q_i2=b_i2;
V_p_i1=q_i1^2;
V_p_i2=q_i2^2;
A=-2/(R_z-Ri)^3;
B=3*(R_z+Ri)/(R_z-Ri)^3;
C=-6*R_z*Ri/(R_z-Ri)^3;
D=Ri^2*(3*R_z-Ri)/(R_z-Ri)^3;
di1=sqrt((x_i-state_friend_1(1))^2+(y_i-state_friend_1(2))^2+(2*(z_i-state_friend_1(3)))^2);
di2=sqrt((x_i-state_friend_2(1))^2+(y_i-state_friend_2(2))^2+(2*(z_i-state_friend_2(3)))^2);
%Define The sigmas
%Sig 1
if di1>=Ri
    sigi1=0;
elseif di1>R_z && di1<Ri
    sigi1=A*di1^3+B*di1^2+C*di1+D;
else
    sigi1=1;
end
%Sig 2
if di2>=Ri
    sigi2=0;
elseif di2>R_z && di2<Ri
    sigi2=A*di2^3+B*di2^2+C*di2+D;
else
    sigi2=1;
end
V_i1=sigi1*V_p_i1;
V_i2=sigi2*V_p_i2;
v_i=V_i0+V_i1+V_i2;
V_i_mdx=v_i/(1+v_i);
x_i=x_i+dx;
%Plus dy
y_i=y_i+dy;
d_s=2*r_i;
R_z=(Ri+2*r_i)/2;
V_i0=(x_i-x_dest)^2+(y_i-y_dest)^2+(2*(z_i-z_dest))^2;
c_i1=(x_i-state_friend_1(1))^2+(y_i-state_friend_1(2))^2+(2*(z_i-state_friend_1(3)))^2-d_s^2;
c_i2=(x_i-state_friend_2(1))^2+(y_i-state_friend_2(2))^2+(2*(z_i-state_friend_2(3)))^2-d_s^2;
b_i1=-100*log(c_i1);
b_i2=-100*log(c_i2);
q_i1=b_i1;
q_i2=b_i2;
V_p_i1=q_i1^2;
V_p_i2=q_i2^2;
A=-2/(R_z-Ri)^3;
B=3*(R_z+Ri)/(R_z-Ri)^3;
C=-6*R_z*Ri/(R_z-Ri)^3;
D=Ri^2*(3*R_z-Ri)/(R_z-Ri)^3;
di1=sqrt((x_i-state_friend_1(1))^2+(y_i-state_friend_1(2))^2+(2*(z_i-state_friend_1(3)))^2);
di2=sqrt((x_i-state_friend_2(1))^2+(y_i-state_friend_2(2))^2+(2*(z_i-state_friend_2(3)))^2);
%Define The sigmas
%Sig 1
if di1>=Ri
    sigi1=0;
elseif di1>R_z && di1<Ri
    sigi1=A*di1^3+B*di1^2+C*di1+D;
else
    sigi1=1;
end
%Sig 2
if di2>=Ri
    sigi2=0;
elseif di2>R_z && di2<Ri
    sigi2=A*di2^3+B*di2^2+C*di2+D;
else
    sigi2=1;
end
V_i1=sigi1*V_p_i1;
V_i2=sigi2*V_p_i2;
v_i=V_i0+V_i1+V_i2;
V_i_pdy=v_i/(1+v_i);
y_i=y_i-dy;
%minus dy
y_i=y_i-dy;
d_s=2*r_i;
R_z=(Ri+2*r_i)/2;
V_i0=(x_i-x_dest)^2+(y_i-y_dest)^2+(2*(z_i-z_dest))^2;
c_i1=(x_i-state_friend_1(1))^2+(y_i-state_friend_1(2))^2+(2*(z_i-state_friend_1(3)))^2-d_s^2;
c_i2=(x_i-state_friend_2(1))^2+(y_i-state_friend_2(2))^2+(2*(z_i-state_friend_2(3)))^2-d_s^2;
b_i1=-100*log(c_i1);
b_i2=-100*log(c_i2);
q_i1=b_i1;
q_i2=b_i2;
V_p_i1=q_i1^2;
V_p_i2=q_i2^2;
A=-2/(R_z-Ri)^3;
B=3*(R_z+Ri)/(R_z-Ri)^3;
C=-6*R_z*Ri/(R_z-Ri)^3;
D=Ri^2*(3*R_z-Ri)/(R_z-Ri)^3;
di1=sqrt((x_i-state_friend_1(1))^2+(y_i-state_friend_1(2))^2+(2*(z_i-state_friend_1(3)))^2);
di2=sqrt((x_i-state_friend_2(1))^2+(y_i-state_friend_2(2))^2+(2*(z_i-state_friend_2(3)))^2);
%Define The sigmas
%Sig 1
if di1>=Ri
    sigi1=0;
elseif di1>R_z && di1<Ri
    sigi1=A*di1^3+B*di1^2+C*di1+D;
else
    sigi1=1;
end
%Sig 2
if di2>=Ri
    sigi2=0;
elseif di2>R_z && di2<Ri
    sigi2=A*di2^3+B*di2^2+C*di2+D;
else
    sigi2=1;
end
V_i1=sigi1*V_p_i1;
V_i2=sigi2*V_p_i2;
v_i=V_i0+V_i1+V_i2;
V_i_mdy=v_i/(1+v_i);
y_i=y_i+dy;
%Plus dz
z_i=z_i+dz;
d_s=2*r_i;
R_z=(Ri+2*r_i)/2;
V_i0=(x_i-x_dest)^2+(y_i-y_dest)^2+(2*(z_i-z_dest))^2;
c_i1=(x_i-state_friend_1(1))^2+(y_i-state_friend_1(2))^2+(2*(z_i-state_friend_1(3)))^2-d_s^2;
c_i2=(x_i-state_friend_2(1))^2+(y_i-state_friend_2(2))^2+(2*(z_i-state_friend_2(3)))^2-d_s^2;
b_i1=-100*log(c_i1);
b_i2=-100*log(c_i2);
q_i1=b_i1;
q_i2=b_i2;
V_p_i1=q_i1^2;
V_p_i2=q_i2^2;
A=-2/(R_z-Ri)^3;
B=3*(R_z+Ri)/(R_z-Ri)^3;
C=-6*R_z*Ri/(R_z-Ri)^3;
D=Ri^2*(3*R_z-Ri)/(R_z-Ri)^3;
di1=sqrt((x_i-state_friend_1(1))^2+(y_i-state_friend_1(2))^2+(2*(z_i-state_friend_1(3)))^2);
di2=sqrt((x_i-state_friend_2(1))^2+(y_i-state_friend_2(2))^2+(2*(z_i-state_friend_2(3)))^2);
%Define The sigmas
%Sig 1
if di1>=Ri
    sigi1=0;
elseif di1>R_z && di1<Ri
    sigi1=A*di1^3+B*di1^2+C*di1+D;
else
    sigi1=1;
end
%Sig 2
if di2>=Ri
    sigi2=0;
elseif di2>R_z && di2<Ri
    sigi2=A*di2^3+B*di2^2+C*di2+D;
else
    sigi2=1;
end
V_i1=sigi1*V_p_i1;
V_i2=sigi2*V_p_i2;
v_i=V_i0+V_i1+V_i2;
V_i_pdz=v_i/(1+v_i);
z_i=z_i-dz;
%minus dx
z_i=z_i-dz;
d_s=2*r_i;
R_z=(Ri+2*r_i)/2;
V_i0=(x_i-x_dest)^2+(y_i-y_dest)^2+(2*(z_i-z_dest))^2;
c_i1=(x_i-state_friend_1(1))^2+(y_i-state_friend_1(2))^2+(2*(z_i-state_friend_1(3)))^2-d_s^2;
c_i2=(x_i-state_friend_2(1))^2+(y_i-state_friend_2(2))^2+(2*(z_i-state_friend_2(3)))^2-d_s^2;
b_i1=-100*log(c_i1);
b_i2=-100*log(c_i2);
q_i1=b_i1;
q_i2=b_i2;
V_p_i1=q_i1^2;
V_p_i2=q_i2^2;
A=-2/(R_z-Ri)^3;
B=3*(R_z+Ri)/(R_z-Ri)^3;
C=-6*R_z*Ri/(R_z-Ri)^3;
D=Ri^2*(3*R_z-Ri)/(R_z-Ri)^3;
di1=sqrt((x_i-state_friend_1(1))^2+(y_i-state_friend_1(2))^2+(2*(z_i-state_friend_1(3)))^2);
di2=sqrt((x_i-state_friend_2(1))^2+(y_i-state_friend_2(2))^2+(2*(z_i-state_friend_2(3)))^2);
%Define The sigmas
%Sig 1
if di1>=Ri
    sigi1=0;
elseif di1>R_z && di1<Ri
    sigi1=A*di1^3+B*di1^2+C*di1+D;
else
    sigi1=1;
end
%Sig 2
if di2>=Ri
    sigi2=0;
elseif di2>R_z && di2<Ri
    sigi2=A*di2^3+B*di2^2+C*di2+D;
else
    sigi2=1;
end
V_i1=sigi1*V_p_i1;
V_i2=sigi2*V_p_i2;
v_i=V_i0+V_i1+V_i2;
V_i_mdz=v_i/(1+v_i);
z_i=z_i+dz;
%Now Calculate V Gradients
der_Vx=(V_i_pdx-V_i_mdx)/(2*dx);
der_Vy=(V_i_pdy-V_i_mdy)/(2*dy);
der_Vz=(V_i_pdz-V_i_mdz)/(2*dz);

dx=dx*100;
dy=dy*100;
dz=dz*100;


Si=zeros(length(x_coord),length(y_coord),length(z_coord));
phi=zeros(length(x_coord),length(y_coord),length(z_coord));
c1=zeros(length(x_coord),length(y_coord),length(z_coord));
c2=zeros(length(x_coord),length(y_coord),length(z_coord));
c3=zeros(length(x_coord),length(y_coord),length(z_coord));
B=zeros(length(x_coord),length(y_coord),length(z_coord));
curr_x_Indx_Num=int64(((1/dx)*(x_i+dx)));
curr_y_Indx_Num=int64(((1/dy)*(y_i+dy)));
curr_z_Indx_Num=int64(((1/dz)*(max(abs(z_coord))+(z_i+dz))));
x_left=max(1,curr_x_Indx_Num-1*(Ri+2*r_i)/dx);
x_right=min(length(x_coord),curr_x_Indx_Num+1*(Ri+2*r_i)/dx);
y_left=max(1,curr_y_Indx_Num-1*(Ri+2*r_i)/dy);
y_right=min(length(y_coord),curr_y_Indx_Num+1*(Ri+2*r_i)/dy);
z_left=max(1,curr_z_Indx_Num-1*(Ri+2*r_i)/dz);
z_right=min(length(z_coord),curr_z_Indx_Num+1*(Ri+2*r_i)/dz);
x_mat = repmat(x_coord(x_left:x_right)',1,length(y_coord(y_left:y_right)),length(z_coord(z_left:z_right)));
y_mat = repmat(y_coord(y_left:y_right),length(x_coord(x_left:x_right)),1,length(z_coord(z_left:z_right)));
z_mat = repmat(z_coord(z_left:z_right),length(x_coord(x_left:x_right)),length(y_coord(y_left:y_right)),1);
phi(x_left:x_right,y_left:y_right,z_left:z_right)...
    =acos((((x_mat-x_i).*cos(yaw_i).*cos(pitch_i)...
    +(y_mat-y_i).*sin(yaw_i).*cos(pitch_i)-...
    (z_mat-z_i).*sin(pitch_i))...
    ./sqrt((x_mat-x_i).^2....
    +(y_mat-y_i).^2....
    +(z_mat-z_i).^2)));

c1(x_left:x_right,y_left:y_right,z_left:z_right)=Ri^2-(repmat(x_coord(x_left:x_right)',1,length(y_coord(y_left:y_right)),length(z_coord(z_left:z_right)))-x_i).^2-(repmat(y_coord(y_left:y_right),length(x_coord(x_left:x_right)),1,length(z_coord(z_left:z_right)))-y_i).^2-(repmat(z_coord(z_left:z_right),length(x_coord(x_left:x_right)),length(y_coord(y_left:y_right)),1)-z_i).^2;
c2(x_left:x_right,y_left:y_right,z_left:z_right)=alpha-phi(x_left:x_right,y_left:y_right,z_left:z_right);
c3(x_left:x_right,y_left:y_right,z_left:z_right)=alpha+phi(x_left:x_right,y_left:y_right,z_left:z_right);
B(x_left:x_right,y_left:y_right,z_left:z_right)=(1/max(0,c1(x_left:x_right,y_left:y_right,z_left:z_right)))+(1/max(0,c2(x_left:x_right,y_left:y_right,z_left:z_right)))+(1/max(0,c3(x_left:x_right,y_left:y_right,z_left:z_right)));
Si(x_left:x_right,y_left:y_right,z_left:z_right)=1./B(x_left:x_right,y_left:y_right,z_left:z_right);
Si(x_left:x_right,y_left:y_right,z_left:z_right)=Si(x_left:x_right,y_left:y_right,z_left:z_right)./max(max(max(Si(x_left:x_right,y_left:y_right,z_left:z_right))));





 Rot_Mat=[cos(pitch_i)*cos(yaw_i),(sin(roll_i)*sin(pitch_i)*cos(yaw_i)-cos(roll_i)*sin(yaw_i)),(cos(roll_i)*sin(pitch_i)*cos(yaw_i)+sin(roll_i)*sin(yaw_i)); ...
         cos(pitch_i)*sin(yaw_i),(sin(roll_i)*sin(pitch_i)*sin(yaw_i)+cos(roll_i)*cos(yaw_i)),(cos(roll_i)*sin(pitch_i)*sin(yaw_i)-sin(roll_i)*cos(yaw_i)); ...
         -sin(pitch_i),(sin(roll_i)*cos(pitch_i)),(cos(roll_i)*cos(pitch_i))];

if sqrt((x_i-x_dest)^2+(y_i-y_dest)^2+(z_i-z_dest)^2)>5
pos_rates=inv(Rot_Mat)*[-.1*der_Vx/sqrt(der_Vx^2+der_Vy^2+der_Vz^2);-.1*der_Vy/sqrt(der_Vx^2+der_Vy^2+der_Vz^2);-.1*der_Vz/sqrt(der_Vx^2+der_Vy^2+der_Vz^2)];
elseif sqrt((x_i-x_dest)^2+(y_i-y_dest)^2+(z_i-z_dest)^2)<=1 && sqrt((x_i-x_dest)^2+(y_i-y_dest)^2+(z_i-z_dest)^2)>1
pos_rates=inv(Rot_Mat)*[-0.01*der_Vx/sqrt(der_Vx^2+der_Vy^2+der_Vz^2);-0.01*der_Vy/sqrt(der_Vx^2+der_Vy^2+der_Vz^2);-0.01*der_Vz/sqrt(der_Vx^2+der_Vy^2+der_Vz^2)];  
else
pos_rates=inv(Rot_Mat)*[-0.01*der_Vx/sqrt(der_Vx^2+der_Vy^2+der_Vz^2);-0.001*der_Vy/sqrt(der_Vx^2+der_Vy^2+der_Vz^2);-0.001*der_Vz/sqrt(der_Vx^2+der_Vy^2+der_Vz^2)];   
end
ui=pos_rates(1);
vi=pos_rates(2);
wi=pos_rates(3);


rates=100*inv([1,sin(roll_i)*tan(pitch_i),cos(roll_i)*tan(pitch_i); ...
           0,cos(roll_i),-sin(roll_i); ...
           0,sin(roll_i)*sec(pitch_i),cos(roll_i)*sec(pitch_i)]) ...
           *[-roll_i;pitch_dest-pitch_i;yaw_dest-yaw_i];
       
%I am only concerned with yaw so just use a fixed rate
if rates(3)>0
rates(3)=2*pi;
end
if rates (3)<0
rates(3)=-2*pi;
end     
        
qi=rates(1);
ri=rates(2);
si=rates(3);
orient_error=[-roll_i;pitch_dest-pitch_i;yaw_dest-yaw_i];
orient_norm=norm(orient_error);

command(1)=real(ui);
command(2)=real(vi);
command(3)=real(wi);
command(4)=real(qi);
command(5)=real(ri);
command(6)=real(si);

if imag(ui)~=0
    fprintf('WTF IMAGINARY STUFF\n')
end


S_i=Si(x_left:x_right,y_left:y_right,z_left:z_right);


end