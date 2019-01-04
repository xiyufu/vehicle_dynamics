function [Vw_long,Vw_lat,beta] = get_wheel_velocities(Vlong,Vlat,dphidt,steer_angle,xref,yref)
 
Vw_long=(Vlong-dphidt*yref)*cos(steer_angle)+(Vlat+dphidt*xref)*sin(steer_angle);
Vw_lat=-(Vlong-dphidt*yref)*sin(steer_angle)+(Vlat+dphidt*xref)*cos(steer_angle);

beta=atan2(Vw_lat,Vw_long);%-steer_angle;

end
