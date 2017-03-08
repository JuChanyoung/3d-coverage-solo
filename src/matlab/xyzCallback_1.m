%------------------------ Define xyzCallback -----------------------%
function xyzCallback_1(src,msg)
    global x_1 y_1 z_1;
    global long_1 lat_1 alt_1;
    global pos_cov_1 type_1;
%     qx_1 = msg.Transform.Rotation.X;
%     qy_1 = msg.Transform.Rotation.Y;
%     qz_1 = msg.Transform.Rotation.Z;
%     qw_1 = msg.Transform.Rotation.W;
    
    long_1 = msg.Longitude;
    lat_1 = msg.Latitude;
    alt_1 = msg.Altitude;
    
    for i=1:9
       pos_cov_1(i) = msg.PositionCovariance(i);
    end
    
    type_1 = msg.PositionCovarianceType;
    
    y_1 =  lat_1-(42.293236100000001);
    x_1 = (long_1)+(83.711706100000001)-1.2126e-5;
    z_1 = alt_1-241.7-2-18-10;
    %aim to have it print -2 to screen for z_1;   
   
   %Larger Ones
%The first multiple converted degrees to meters, the second meters to matlab coordinates, the third is a shift for the origin.
    
%     x_1 = ((x_1)*82469.093)*(200/30)+100;
%     y_1 = ((y_1)*111078.9538)*(200/30)+100;
%     z_1 = (z_1)-100;
    
    y_1 = ((y_1)*111078.9538); % convert to meters only
    x_1 = ((x_1)*82469.093);
    z_1 = (z_1);
    
end
        
%---------------------------- END ------------------------------------%
