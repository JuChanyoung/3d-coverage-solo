%------------------------ Define xyzCallback -----------------------%
function xyzCallback_2(src,msg)    
global x_2 y_2 z_2;
    global long_2 lat_2 alt_2;
    global pos_cov_2 type_2;
%     qx_1 = msg.Transform.Rotation.X;
%     qy_1 = msg.Transform.Rotation.Y;
%     qz_1 = msg.Transform.Rotation.Z;
%     qw_1 = msg.Transform.Rotation.W;
    
    long_2 = msg.Longitude;
    lat_2 = msg.Latitude;
    alt_2 = msg.Altitude;
    
    for i=1:9
       pos_cov_2(i) = msg.PositionCovariance(i);
    end
    
    type_2 = msg.PositionCovarianceType;
    
    x_2 =  lat_2-(42.293233218565383);
    y_2 = -(long_2)-(83.711667016455664);
    z_2 = alt_2-239-2;    
    
    %True/Zhipeng Transformations
   % x_1 = (x_1+1.42)*200/1.8;
   % y_1 = (y_1+0.12)*200/1.8;
   % z_1 = (z_1-0.16)*100/1.5-100;
    
    %New Will Transformations (teenier)
    
    %x_1 = (x_1+1.27)*200/1.5;
    %y_1 = (y_1-.03)*200/1.5;
    %z_1 = (z_1-0.2)*100/1.5-100;
    
        %Newer Will (teenierer)
   %  x_1 = (x_1+1.22)*200/1.4;
   %  y_1 = (y_1-.08)*200/1.4;
   %  z_1 = (z_1-0.2)*100/1.5-100;
   
    x_2 = ((x_2)*111078.9538); % convert to meters only
    y_2 = ((y_2)*82469.093);
    z_2 = (z_2);
end
        
%---------------------------- END ------------------------------------%
