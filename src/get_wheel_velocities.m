function [Vfl, Vfr, Vrl, Vrr, alpha_fl, alpha_fr, alpha_rl, alpha_rr] = get_wheel_velocities(V_long, V_lat, phi_dot, delta, vehicle_param)
%get_wheel_velocities calculate wheel velocites and slip angles for friction caculation
%   phi_dot, phi - rotation in global fram
%   input - input parameters
%   vehicle_param - vehicle parameters
%   V_long - longitudinal velocity in car frame
%   V_lat - lateral velocity in car frame
%   delta - steering angle

a = vehicle_param.a;
b = vehicle_param.b;
t = vehicle_param.Df;
% wheel velocites in car frame
Vxfl = (V_long - phi_dot*t/2)*cos(delta) + (V_lat + phi_dot*a)*sin(delta);
Vyfl = -(V_long - phi_dot*t/2)*sin(delta) + (V_lat + phi_dot*a)*cos(delta);
Vfl = [Vxfl; Vyfl];
alpha_fl = atan2(Vyfl, Vxfl);

Vxfr = (V_long + phi_dot*t/2)*cos(delta) + (V_lat + phi_dot*a)*sin(delta);
Vyfr = -(V_long + phi_dot*t/2)*sin(delta) + (V_lat +phi_dot*a)*cos(delta);
Vfr = [Vxfr; Vyfr];
alpha_fr = atan2(Vyfr, Vxfr);

Vxrl = V_long - phi_dot*t/2;
Vyrl = V_lat - phi_dot*b;
Vrl = [Vxrl; Vyrl];
alpha_rl = atan2(Vyrl, Vxrl);

Vxrr = V_long + phi_dot*t/2;
Vyrr = V_lat - phi_dot*b;
Vrr = [Vxrr; Vyrr];
alpha_rr = atan2(Vyrr, Vxrr);

end

