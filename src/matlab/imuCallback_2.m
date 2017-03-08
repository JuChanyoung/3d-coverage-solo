%------------------------ Define imuCallback -----------------------%
function imuCallback_2(src,msg)
    global qx_2 qy_2 qz_2 qw_2 orient_cov_2;
    global wx_2 wy_2 wz_2 angular_cov_2;
    global ax_2 ay_2 az_2 accel_cov_2;
    
    qx_2 = msg.Orientation.X;
    qy_2 = msg.Orientation.Y;
    qz_2 = msg.Orientation.Z;
    qw_2 = msg.Orientation.W;
  
    
    wx_2 = msg.AngularVelocity.X;
    wy_2 = msg.AngularVelocity.Y;
    wz_2 = msg.AngularVelocity.Z;
    
    ax_2 = msg.LinearAcceleration.X;
    ay_2 = msg.LinearAcceleration.Y;
    az_2 = msg.LinearAcceleration.Z;
    
%     for i=1:9
%         angular_cov_2(i) = msg.AngularVelocityCovariance(i);
%         accel_cov_2(i) = msg.LinearAccelerationCovariance(i);
%         orient_cov_2(i) = msg.OrientationCovariance(i);
%     end
  
end
        
%---------------------------- END ------------------------------------%
