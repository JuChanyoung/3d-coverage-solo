function [command,S_i]=Local_Cov2(Q,state_me,x_coord,y_coord,z_coord,dx,dy,dz,K_u,K_v,K_w,K_r,K_s,Ri,alpha,c_star,r_i)

x_i=state_me(1);
y_i=state_me(2);
z_i=state_me(3);
pitch_i=state_me(4);
yaw_i=state_me(5);
roll_i=state_me(6);




curr_x_Indx_Num=int64(((1/dx)*(x_i+dx)));
curr_y_Indx_Num=int64(((1/dy)*(y_i+dy)));
%This one is different. Note that z is defined down and the array runs
%backwards. Look at the z_coord compared to x_coord if you don't understand
curr_z_Indx_Num=int64(((1/dz)*(max(abs(z_coord))+(z_i+dz))));


%THIS BLOCK HAS CHANGED 6/9/16. Note the 1's instead of 1.5's. Box is now
%2Rx2Rx2R instead of 3R etc
x_left=max(1,curr_x_Indx_Num-1*(Ri+2*r_i)/dx);
x_right=min(length(x_coord),curr_x_Indx_Num+1*(Ri+2*r_i)/dx);
y_left=max(1,curr_y_Indx_Num-1*(Ri+2*r_i)/dy);
y_right=min(length(y_coord),curr_y_Indx_Num+1*(Ri+2*r_i)/dy);
z_left=max(1,curr_z_Indx_Num-1*(Ri+2*r_i)/dz);
z_right=min(length(z_coord),curr_z_Indx_Num+1*(Ri+2*r_i)/dz);

x_mat = repmat(x_coord(x_left:x_right)',1,length(y_coord(y_left:y_right)),length(z_coord(z_left:z_right)));
y_mat = repmat(y_coord(y_left:y_right),length(x_coord(x_left:x_right)),1,length(z_coord(z_left:z_right)));
z_mat = repmat(z_coord(z_left:z_right),length(x_coord(x_left:x_right)),length(y_coord(y_left:y_right)),1);
phi...
    =acos((((x_mat-x_i).*cos(yaw_i).*cos(pitch_i)...
    +(y_mat-y_i).*sin(yaw_i).*cos(pitch_i)-...
    (z_mat-z_i).*sin(pitch_i))...
    ./sqrt((x_mat-x_i).^2....
    +(y_mat-y_i).^2....
    +(z_mat-z_i).^2)));

c1=Ri^2-(repmat(x_coord(x_left:x_right)',1,length(y_coord(y_left:y_right)),length(z_coord(z_left:z_right)))-x_i).^2-(repmat(y_coord(y_left:y_right),length(x_coord(x_left:x_right)),1,length(z_coord(z_left:z_right)))-y_i).^2-(repmat(z_coord(z_left:z_right),length(x_coord(x_left:x_right)),length(y_coord(y_left:y_right)),1)-z_i).^2;
c2=alpha-phi;
c3=alpha+phi;
B=(1/max(0,c1))+(1/max(0,c2))+(1/max(0,c3));
Si=1./B;
Si=Si./max(max(max(Si)));

Common_term_n1=2.*alpha.*x_i.^2 - 2.*Ri.^2.*alpha + 2.*alpha.*x_mat.^2 + 2.*alpha.*y_i.^2 + 2.*alpha.*y_mat.^2 + 2.*alpha.*z_i.^2 + 2.*alpha.*z_mat.^2;
Common_term_1=acos(z_i.*sin(pitch_i) - z_mat.*sin(pitch_i) - x_i.*cos(pitch_i).*cos(yaw_i) + x_mat.*cos(pitch_i).*cos(yaw_i) - y_i.*cos(pitch_i).*sin(yaw_i) + y_mat.*cos(pitch_i).*sin(yaw_i));
Common_term_2=Common_term_n1 - alpha.^2;
Common_term_3=acos(sin(pitch_i).*(z_i - z_mat) - cos(pitch_i).*cos(yaw_i).*(x_i - x_mat) - cos(pitch_i).*sin(yaw_i).*(y_i - y_mat));
Common_term_4=(Common_term_n1 + Common_term_3.^2 - alpha.^2 - 4.*alpha.*x_i.*x_mat - 4.*alpha.*y_i.*y_mat - 4.*alpha.*z_i.*z_mat);
Common_term_5=(1 - (cos(pitch_i).*cos(yaw_i).*(x_i - x_mat) - sin(pitch_i).*(z_i - z_mat) + cos(pitch_i).*sin(yaw_i).*(y_i - y_mat)).^2);
Common_term_6=((x_i - x_mat).^2 + (y_i - y_mat).^2 + (z_i - z_mat).^2 - Ri.^2);
Common_term_7=- 2.*Ri.^2.*alpha - alpha.^2 + 2.*alpha.*x_i.^2 - 4.*alpha.*x_i.*x_mat + 2.*alpha.*x_mat.^2 + 2.*alpha.*y_i.^2 - 4.*alpha.*y_i.*y_mat + 2.*alpha.*y_mat.^2 + 2.*alpha.*z_i.^2 - 4.*alpha.*z_i.*z_mat + 2.*alpha.*z_mat.^2;
Common_term_8=(Common_term_2 + Common_term_1.^2 - 4.*alpha.*x_i.*x_mat - 4.*alpha.*y_i.*y_mat - 4.*alpha.*z_i.*z_mat);
Common_term_9=(Common_term_7 + Common_term_3.^2).^2;
Common_term_10=(- Ri.^2 + x_i.^2 - 2.*x_i.*x_mat + x_mat.^2 + y_i.^2 - 2.*y_i.*y_mat + y_mat.^2 + z_i.^2 - 2.*z_i.*z_mat + z_mat.^2);
Common_term_11=(alpha.^2 - Common_term_1.^2);
Common_term_12=(Common_term_5.^(1./2).*Common_term_4);
Common_term_13=alpha + Common_term_3;
Common_term_14=alpha - Common_term_3;
Common_term_15=(Common_term_5.^(1./2).*Common_term_9);
Common_term_16=(Common_term_13).*(Common_term_14);
%toc
%X derivative
Si_check=Si~=0;
Si_check=double(Si_check);

Si_dx=((2.*x_i - 2.*x_mat).*Common_term_11)./Common_term_8 - (Common_term_16.*(4.*alpha.*x_i - 4.*alpha.*x_mat + (2.*Common_term_3.*cos(pitch_i).*cos(yaw_i))./Common_term_5.^(1./2)).*Common_term_6)./Common_term_9 + (cos(pitch_i).*cos(yaw_i).*(Common_term_14).*Common_term_6)./Common_term_12 - (cos(pitch_i).*cos(yaw_i).*(Common_term_13).*Common_term_6)./Common_term_12;
Si_dx=Si_check.*Si_dx;

%Y derivative

Si_dy=((2.*y_i - 2.*y_mat).*Common_term_11)./Common_term_8 - (Common_term_16.*(4.*alpha.*y_i - 4.*alpha.*y_mat + (2.*Common_term_3.*cos(pitch_i).*sin(yaw_i))./Common_term_5.^(1./2)).*Common_term_6)./Common_term_9 + (cos(pitch_i).*sin(yaw_i).*(Common_term_14).*Common_term_6)./Common_term_12 - (cos(pitch_i).*sin(yaw_i).*(Common_term_13).*Common_term_6)./Common_term_12;

Si_dy=Si_check.*Si_dy;



%Z derivative

Si_dz=((2.*z_i - 2.*z_mat).*Common_term_11)./Common_term_8 + (sin(pitch_i).*(Common_term_13).*Common_term_6)./Common_term_12 - (sin(pitch_i).*(Common_term_14).*Common_term_6)./Common_term_12 + (Common_term_16.*(4.*alpha.*z_mat - 4.*alpha.*z_i + (2.*Common_term_3.*sin(pitch_i))./Common_term_5.^(1./2)).*Common_term_6)./Common_term_9;
Si_dz=Si_check.*Si_dz;

%Yaw derivative

Si_dyaw=-(4.*alpha.*Common_term_3.*cos(pitch_i).*(y_i.*cos(yaw_i) - y_mat.*cos(yaw_i) - x_i.*sin(yaw_i) + x_mat.*sin(yaw_i)).*Common_term_10.^2)./Common_term_15;
Si_dyaw=Si_check.*Si_dyaw;



%Pitch derivative

Si_dpitch=(4.*alpha.*Common_term_3.*(z_i.*cos(pitch_i) - z_mat.*cos(pitch_i) + x_i.*cos(yaw_i).*sin(pitch_i) - x_mat.*cos(yaw_i).*sin(pitch_i) + y_i.*sin(pitch_i).*sin(yaw_i) - y_mat.*sin(pitch_i).*sin(yaw_i)).*Common_term_10.^2)./Common_term_15;
Si_dpitch=Si_check.*Si_dpitch;


ai0=trapz(trapz(trapz(6.*max(0,c_star-Q).*...
    Si.^2,1),2),3).*dx.*dy.*dz;
ai1=trapz(trapz(trapz((3.*max(0,c_star-Q).^2.*(...
    Si_dx.*...
    cos(pitch_i).*cos(yaw_i)...
+Si_dy.*cos(pitch_i).*sin(yaw_i)...
+Si_dz.*(-sin(pitch_i)))),1),2),3).*dx.*dy.*dz;
ai2=trapz(trapz(trapz((3.*max(0,c_star-Q).^2).*(Si_dx.*(sin(roll_i).*sin(pitch_i).*cos(yaw_i)-cos(roll_i).*sin(yaw_i))+Si_dy.*(sin(roll_i).*sin(pitch_i).*sin(yaw_i)+cos(roll_i).*cos(yaw_i))+Si_dz.*(sin(roll_i).*cos(pitch_i))),1),2),3).*dx.*dy.*dz;
ai3=trapz(trapz(trapz((3.*max(0,c_star-Q).^2).*(Si_dx.*(cos(roll_i).*sin(pitch_i).*cos(yaw_i)+sin(roll_i).*sin(yaw_i))+Si_dy.*(cos(roll_i).*sin(pitch_i).*sin(yaw_i)-sin(roll_i).*cos(yaw_i))+Si_dz.*(cos(roll_i).*cos(pitch_i))),1),2),3).*dx.*dy.*dz;
ai4=trapz(trapz(trapz((3.*max(0,c_star-Q).^2).*(Si_dyaw.*(sin(roll_i).*sec(pitch_i))+Si_dpitch.*cos(roll_i)),1),2),3).*dx.*dy.*dz;
ai5=trapz(trapz(trapz((3.*max(0,c_star-Q).^2).*(Si_dyaw.*(cos(roll_i).*sec(pitch_i))+Si_dpitch.*(-sin(roll_i))),1),2),3).*dx.*dy.*dz;

ui=K_u*ai1;
vi=K_v*ai2;
wi=K_w*ai3;
qi=0;
ri=K_r*ai4;
si=K_s*ai5;

command(1)=ui;
command(2)=vi;
command(3)=wi;
command(4)=qi;
command(5)=ri;
command(6)=si;

S_i=Si;
end
