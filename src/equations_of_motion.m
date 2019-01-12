function    [dX, data] = equations_of_motion(vehicle_param, inputs, X, dX_old, k)
%
% This file solves the equations of motion with respect to the acceleration
% in order to determine dX. Other interesting variables can be stored in
% 'data'. 
%
%   dX = [velocity;
%         acceleration]
%          

% X = [x, y, phi, phifr, phifl, phirr, phirl, x_dot, y_dot, phi_dot, phifr_dot, phifl_dot, phirr_dot, phirl_dot]


dX = zeros(vehicle_param.n_dofs*2,1);

% Velocity equals the velocity from the previous time-step:
dX(1:vehicle_param.n_dofs) = X(vehicle_param.n_dofs+1:end);

%% Perform projections on velocities:
% X = [x, y, phi, phifr, phifl, phirr, phirl, x_dot, y_dot, phi_dot, phifr_dot, phifl_dot, phirr_dot, phirl_dot]
phi = X(3);
delta = inputs.delta(k);
% theta = inputs.inclin_angle;
a = vehicle_param.a;
b = vehicle_param.b;
Iw = vehicle_param.I_w;
Izz = vehicle_param.Izz;
r = vehicle_param.r_w;
t = vehicle_param.Df;
m = vehicle_param.M;
%% Calculate tyre forces:
% Modify this function so that ds and dalpha are returned.
[Ffl, Ffr, Frl, Frr, F_aero,alpha, slips, ds, dalpha] = get_tyre_forces(vehicle_param, inputs, X, dX_old, k, inputs.relaxation_disable);
Fxfl = Ffl(1);
Fyfl = Ffl(2);
Fzfl = Ffl(3);
Fxfr = Ffr(1);
Fyfr = Ffr(2);
Fzfr = Ffr(3);
Fxrl = Frl(1);
Fyrl = Frl(2);
Fzrl = Frl(3);
Fxrr = Frr(1);
Fyrr = Frr(2);
Fzrr = Frr(3);
%% Determine accelerations:
% IN GLOBAL
% X = [x, y, phi, phifr, phifl, phirr, phirl, x_dot, y_dot, phi_dot, phifr_dot, phifl_dot, phirr_dot, phirl_dot]
% x
Fxg = (Fxfl+Fxfr)*cos(phi+delta) - (Fyfl+Fyfr)*sin(phi+delta) + (Fxrl+Fxrr)*cos(phi) - (Fyrl+Fyrr)*sin(phi) - F_aero*cos(phi);
dX(vehicle_param.n_dofs + 1) = Fxg/m;
% y
Fyg = (Fxfl+Fxfr)*sin(phi+delta) + (Fyfl+Fyfr)*cos(phi+delta) + (Fxrl+Fxrr)*sin(phi) + (Fyrl+Fyrr)*cos(phi) - F_aero*sin(phi);
dX(vehicle_param.n_dofs + 2) = Fyg/m;
% yaw
moment = (-(Fxfl*cos(delta)-Fyfl*sin(delta))+ (Fxfr*cos(delta)-Fyfr*sin(delta)) - Fxrl + Fxrr )*t/2 ...
                    + (Fxfl*sin(delta)+Fyfl*cos(delta)+Fyfr*cos(delta)+Fxfr*sin(delta))*a + (-Fyrl - Fyrr)*b;
dX(vehicle_param.n_dofs + 3) = moment/Izz;
% wheels
if inputs.tc_disable == 1
    Tfl = inputs.T(1,k);
    Tfr = inputs.T(2,k);
    Trl = inputs.T(3,k);
    Trr = inputs.T(4,k);
    if inputs.small_torque_disable == 0 && k > 1000
        Trl = 10;
        Trr = 10;
    end
else
%     if k == 1000
%         k;
%     end
    [dTfl,dTfr,dTrl,dTrr] = torque_vec(inputs, 1000, alpha, Ffl, Ffr, Frl, Frr, vehicle_param,k);
    Tfl = inputs.T(1,k) + dTfl;
    Tfr = inputs.T(2,k) + dTfr;
    Trl = inputs.T(3,k) + dTrl;
    Trr = inputs.T(4,k) + dTrr;
end

dX(vehicle_param.n_dofs + 4) = (Tfr - r*Fxfr)/Iw;
dX(vehicle_param.n_dofs + 5) = (Tfl - r*Fxfl)/Iw;
dX(vehicle_param.n_dofs + 6) = (Trr - r*Fxrr)/Iw;
dX(vehicle_param.n_dofs + 7) = (Trl - r*Fxrl)/Iw;
% if Trr-r*Fxrr < 0 && dX(1)>dX(6)*r 
%     k;
% end
dX(vehicle_param.n_dofs + 8:vehicle_param.n_dofs + 15) = [ds';dalpha'];
% Additional interesting data can be stored in data:
% lateral in car frame
% a_lat = -dX(vehicle_param.n_dofs + 1)*sin(phi) + dX(vehicle_param.n_dofs + 2)*cos(phi);
% data = [a_lat; alpha';Frl(3)];
% if delta > 0 && mod(k, 100) == 0
%     k;
% end
data = [Fxfl;Fyfl;Fzfl;Fxfr;Fyfr;Fzfr;Fxrl;Fyrl;Fzrl;Fxrr;Fyrr;Fzrr;slips';alpha';Trl;Trr];

