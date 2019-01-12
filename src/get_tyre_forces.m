function [Ffl, Ffr, Frl, Frr, F_aero, alpha, slips, ds, dalpha] = get_tyre_forces(vehicle_param, inputs, X, dX_old, k, relaxation_disable)
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

%% Calculate longitudinal slip:
% - sign in front of omega because of sign convention
[sfl, sign_Fxfl] = calc_slip(Vfl(1), omega_fl*r);
[sfr, sign_Fxfr] = calc_slip(Vfr(1), omega_fr*r);
[srl, sign_Fxrl] = calc_slip(Vrl(1), omega_rl*r);
[srr, sign_Fxrr] = calc_slip(Vrr(1), omega_rr*r);

% % test
if k==2000
    k;
end

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
ds = zeros(1,4);
dalpha = zeros(1,4);
if relaxation_disable == 0
    % override s and alpha. Not efficient though...
    % X(8:15) ~ sfl, sfr, srl, srr, alpha_fl, alpha_fr, alpha_rl, alpha_rr
    sfl = X(8);
    sfr = X(9);
    srl = X(10);
    srr = X(11);
    alpha_fl = X(12);
    alpha_fr = X(13);
    alpha_rl = X(14);
    alpha_rr = X(15);
%     kappa_f = Vsxf/Vxf;
%     alpha_f = atan2(Vsyf,Vxf);
%     
%     kappa_r = Vsxr/Vxr;
%     alpha_r = atan2(Vsyr,Vxr);
% dx(9)  = (-Vxf*x(9)  + Vsxf)/param.sigma_x; % Tyre relaxation longitudinal - front
% dx(10) = (-Vxf*x(10) + Vsyf)/param.sigma_y; % Tyre relaxation lateral - front

%     ds(1) = (-Vfl(1)*sfl + max(r*omega_fl-Vfl(1), Vfl(1)-r*omega_fl))/vehicle_param.sigma_x;
%     ds(2) = (-Vfr(1)*sfr + max(r*omega_fr-Vfr(1), Vfr(1)-r*omega_fr))/vehicle_param.sigma_x;
%     ds(3) = (-Vrl(1)*srl + max(r*omega_rl-Vrl(1), Vrl(1)-r*omega_rl))/vehicle_param.sigma_x;
%     ds(4) = (-Vrr(1)*srr + max(r*omega_rr-Vrr(1), Vrr(1)-r*omega_rr))/vehicle_param.sigma_x;
    ds(1) = (-Vfl(1)*sfl +r*omega_fl-Vfl(1))/vehicle_param.sigma_x;
    ds(2) = (-Vfr(1)*sfr + r*omega_fr-Vfr(1))/vehicle_param.sigma_x;
    ds(3) = (-Vrl(1)*srl + r*omega_rl-Vrl(1))/vehicle_param.sigma_x;
    ds(4) = (-Vrr(1)*srr + r*omega_rr-Vrr(1))/vehicle_param.sigma_x;
%     sfl = sfl + ds(1)*inputs.inner_dt;
%     sfr = sfr + ds(2)*inputs.inner_dt;
%     srl = srl + ds(3)*inputs.inner_dt;
%     srr = srr + ds(4)*inputs.inner_dt;
    
    dalpha(1) = (-Vfl(1)*alpha_fl + Vfl(2))/vehicle_param.sigma_y;
    dalpha(2) = (-Vfr(1)*alpha_fr + Vfr(2))/vehicle_param.sigma_y;
    dalpha(3) = (-Vrl(1)*alpha_rl + Vrl(2))/vehicle_param.sigma_y;
    dalpha(4) = (-Vrr(1)*alpha_rr + Vrr(2))/vehicle_param.sigma_y;
%     alpha_fr = alpha_fr + dalpha(1)*inputs.inner_dt;
%     alpha_fl = alpha_fl + dalpha(2)*inputs.inner_dt;
%     alpha_rl = alpha_rl + dalpha(3)*inputs.inner_dt;
%     alpha_rr = alpha_rr + dalpha(4)*inputs.inner_dt;
    

%     sign_Fxfl = sign(r*omega_fl-Vfl(1));
%     sign_Fxfr = sign(r*omega_fr-Vfr(1));
%     sign_Fxrl = sign(r*omega_rl-Vrl(1));
%     sign_Fxrr = sign(r*omega_rr-Vrr(1));
    sign_Fxfl = sign(sfl);
    sign_Fxfr = sign(sfr);
    sign_Fxrl = sign(srl);
    sign_Fxrr = sign(srr);
end
alpha = [alpha_fl, alpha_fr, alpha_rl, alpha_rr];
slips = [sfl,sfr,srl,srr];

Ffl = tyre_model_Dugoff(F_nfl, alpha_fl, sfl, vehicle_param.mu, vehicle_param.Cx_f, vehicle_param.Cy_f, sign_Fxfl);
Ffr = tyre_model_Dugoff(F_nfr, alpha_fr, sfr, vehicle_param.mu, vehicle_param.Cx_f, vehicle_param.Cy_f, sign_Fxfr);
Frl = tyre_model_Dugoff(F_nrl, alpha_rl, srl, vehicle_param.mu, vehicle_param.Cx_r, vehicle_param.Cy_r, sign_Fxrl);
Frr = tyre_model_Dugoff(F_nrr, alpha_rr, srr, vehicle_param.mu, vehicle_param.Cx_r, vehicle_param.Cy_r, sign_Fxrr);

% Ffl = tyre_model_Dugoff(F_nfl, alpha_fl, abs(sfl), vehicle_param.mu, vehicle_param.Cx_f, vehicle_param.Cy_f,0);
% Ffr = tyre_model_Dugoff(F_nfr, alpha_fr, abs(sfr), vehicle_param.mu, vehicle_param.Cx_f, vehicle_param.Cy_f,0);
% Frl = tyre_model_Dugoff(F_nrl, alpha_rl, abs(srl), vehicle_param.mu, vehicle_param.Cx_r, vehicle_param.Cy_r,0);
% Frr = tyre_model_Dugoff(F_nrr, alpha_rr, abs(srr), vehicle_param.mu, vehicle_param.Cx_r, vehicle_param.Cy_r,0);
% Ffl(1) = -sign_Fxfl*Ffl(1);
% Ffr(1) =-sign_Fxfr*Ffr(1);
% Frl(1) = -sign_Fxrl*Frl(1);
% Frr(1) = -sign_Fxrr*Frr(1);

end

