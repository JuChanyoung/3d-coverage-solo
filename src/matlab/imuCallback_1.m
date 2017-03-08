%------------------------ Define imuCallback -----------------------%
function imuCallback_1(src,msg)
    global qx_1 qy_1 qz_1 qw_1 orient_cov_1;
    global wx_1 wy_1 wz_1 angular_cov_1;
    global ax_1 ay_1 az_1 accel_cov_1;
    
    qx_1 = msg.Orientation.X;
    qy_1 = msg.Orientation.Y;
    qz_1 = msg.Orientation.Z;
    qw_1 = msg.Orientation.W;
    
    wx_1 = msg.AngularVelocity.X;
    wy_1 = msg.AngularVelocity.Y;
    wz_1 = msg.AngularVelocity.Z;
    
    ax_1 = msg.LinearAcceleration.X;
    ay_1 = msg.LinearAcceleration.Y;
    az_1 = msg.LinearAcceleration.Z;
    
%     for i=1:9
%         angular_cov_1(i) = msg.AngularVelocityCovariance(i);
%         accel_cov_1(i) = msg.LinearAccelerationCovariance(i);
%         orient_cov_1(i) = msg.OrientationCovariance(i);
%     end
  
end
        
%---------------------------- END ------------------------------------%
