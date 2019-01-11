function [Ffl, Ffr, Frl, Frr, F_aero, alpha, slips] = get_tyre_forces(vehicle_param, inputs, X, dX_old, k)
%get_tyre_forces calculate tyre forces for 4-wheel model
% Velocity equals the velocity from the previous time-step:
dX(1:vehicle_param.n_dofs) = X(vehicle_param.n_dofs+1:end);

%% Perform velocity projections on velocities:
m = vehicle_param.M;
a = vehicle_param.a;
b = vehicle_param.b;
t = vehicle_param.Df;
theta = inputs.inclin_angle;
h = vehicle_param.h;
ha = vehicle_param.h_aero;
r = vehicle_param.r_w;
g= 9.81;
% X = [x, y, phi, phifr, phifl, phirr, phirl, x_dot, y_dot, phi_dot, phifr_dot, phifl_dot, phirr_dot, phirl_dot]
phi = X(3);
x_dot = X(vehicle_param.n_dofs + 1);
y_dot = X(vehicle_param.n_dofs + 2);
dphidt = X(vehicle_param.n_dofs+3);
omega_fr = X(vehicle_param.n_dofs + 4);
omega_fl = X(vehicle_param.n_dofs + 5);
omega_rr = X(vehicle_param.n_dofs + 6);
omega_rl = X(vehicle_param.n_dofs +7);

delta = inputs.delta(k);
 
%% velocities in car frame
V_long = x_dot*cos(phi) + y_dot*sin(phi);
V_lat = -x_dot*sin(phi) + y_dot*cos(phi);

%% Calculate aerodynamics forces:
% don't put sign here
F_aero = V_long^2*vehicle_param.rho_air*vehicle_param.Cv*vehicle_param.A_front/2;
%% Calculate wheel velocities:
[Vfl, Vfr, Vrl, Vrr, alpha_fl, alpha_fr, alpha_rl, alpha_rr] = get_wheel_velocities(V_long, V_lat, dphidt, delta, vehicle_param);
alpha = [alpha_fl, alpha_fr, alpha_rl, alpha_rr];
%% Calculate longitudinal slip:
% - sign in front of omega because of sign convention
[sfl, sign_Fxfl] = calc_slip(Vfl(1), omega_fl*r);
[sfr, sign_Fxfr] = calc_slip(Vfr(1), omega_fr*r);
[srl, sign_Fxrl] = calc_slip(Vrl(1), omega_rl*r);
[srr, sign_Fxrr] = calc_slip(Vrr(1), omega_rr*r);

slips = [sfl;sfr;srl;srr];
% % test
% if sign(V_longf*omega_f) < 0
%     flag = 1;
% end

%% Calculate normal forces:
ax = dX_old(vehicle_param.n_dofs + 1)*cos(phi) + dX_old(vehicle_param.n_dofs +2)*sin(phi);
ay = -dX_old(vehicle_param.n_dofs + 1)*sin(phi) + dX_old(vehicle_param.n_dofs + 2)*cos(phi);

% The stiffness of the anti-roll bar is needed to get exact load
% distribution, here I simply allocate the load evenly
F_nf = (b*m*g*cos(theta) - F_aero*ha - h*m*g*sin(theta) - m*ax*h)/(a+b);
F_nr = (a*m*g*cos(theta) + F_aero*ha  + h*m*g*sin(theta) + m*ax*h)/(a+b);
delta_n = m*ay*h/t/2;
F_nfl = F_nf/2 - delta_n;
F_nfr = F_nf/2 + delta_n;
F_nrl = F_nr/2 - delta_n;
F_nrr = F_nr/2 + delta_n;
%% Calculate tyre forces:
Ffl = tyre_model_Dugoff(F_nfl, alpha_fl, sfl, vehicle_param.mu, vehicle_param.Cx_f, vehicle_param.Cy_f, sign_Fxfl);
Ffr = tyre_model_Dugoff(F_nfr, alpha_fr, sfr, vehicle_param.mu, vehicle_param.Cx_f, vehicle_param.Cy_f, sign_Fxfr);
Frl = tyre_model_Dugoff(F_nrl, alpha_rl, srl, vehicle_param.mu, vehicle_param.Cx_r, vehicle_param.Cy_r, sign_Fxrl);
Frr = tyre_model_Dugoff(F_nrr, alpha_rr, srr, vehicle_param.mu, vehicle_param.Cx_r, vehicle_param.Cy_r, sign_Fxrr);

end

