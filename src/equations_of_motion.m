function    [dX, data] = equations_of_motion(vehicle_param, inputs, X, dX_old, k)
%
% This file solves the equations of motion with respect to the acceleration
% in order to determine dX. Other interesting variables can be stored in
% 'data'. 
%
%   dX = [velocity;
%         acceleration]
%          

% X = [x, y, phi, phi_f, phi_r, x_dot, y_dot, phi_dot, omega_f, omega_r]


dX = zeros(vehicle_param.n_dofs*2,1);

% Velocity equals the velocity from the previous time-step:
dX(1:vehicle_param.n_dofs) = X(vehicle_param.n_dofs+1:end);

%% Perform projections on velocities:
phi = X(3);
dphidt = X(vehicle_param.n_dofs+3);
steer_angle = inputs.delta(k);

rot_global_to_car=[cos(phi),sin(phi);-sin(phi),cos(phi)];
rot_car_global=[cos(phi),-sin(phi);sin(phi),cos(phi)];
rot_fwheel_to_global = [cos(phi+steer_angle), -sin(phi+steer_angle); sin(phi+steer_angle), cos(phi+steer_angle)];
 

V_car=rot_global_to_car*transpose([X(vehicle_param.n_dofs+1),X(vehicle_param.n_dofs+2)]);
a_car_old=rot_global_to_car*transpose([dX_old(vehicle_param.n_dofs+1),dX_old(vehicle_param.n_dofs+2)]);


% Vlong=V_car(1);
% Vlat=V_car(2);

%% Calculate aerodynamics forces:

F_aero=(V_car(1).^2*vehicle_param.rho_air*vehicle_param.Cv*vehicle_param.A_front)/2;

%% Calculate tyre forces:

[F_wfl, F_wfr, F_wrl, F_wrr, data] = get_tyre_forces(a_car_old,V_car,dphidt,steer_angle,F_aero,vehicle_param,inputs,X,k); 

%Usage: F_long = F(1); F_lat = F(2)


F_wfl=[F_wfl(1) F_wfl(2)];
F_wfr=[F_wfr(1) F_wfr(2)];
F_wrl=[F_wrl(1) F_wrl(2)];
F_wrr=[F_wrr(1) F_wrr(2)];

%% Determine accelerations:

% Acceleration is determined from the force-equilibrium of the system:

[F_wfl_global]=rot_fwheel_to_global*transpose(F_wfl);
[F_wfr_global]=rot_fwheel_to_global*transpose(F_wfr);
[F_wrl_global]=rot_car_global*transpose(F_wrl);
[F_wrr_global]=rot_car_global*transpose(F_wrr);

F_aero_global=rot_car_global*transpose([F_aero 0]);

%a_long=((F_wfl(1)+F_wfr(1))*cos(steer_angle)-(F_wfl(2)+F_wfr(2))*sin(steer_angle)+F_wrl(1)+F_wrr(1)-F_aero)/vehicle_param.M;
%a_lat=((F_wfl(1)+F_wfr(1))*sin(steer_angle)+(F_wfl(2)+F_wfr(2))*cos(steer_angle)+F_wrl(2)+F_wrr(2))/vehicle_param.M;

%a_car=rot_car_to_global*transpose([a_long,a_lat]);

dX(vehicle_param.n_dofs+1)=(F_wfl_global(1)+F_wfr_global(1)+F_wrl_global(1)+F_wrr_global(1)-F_aero_global(1))/vehicle_param.M;                     %a_car(1);
dX(vehicle_param.n_dofs+2)=(F_wfl_global(2)+F_wfr_global(2)+F_wrl_global(2)+F_wrr_global(2)-F_aero_global(2))/vehicle_param.M;                      %a_car(2);
dX(vehicle_param.n_dofs+3)=(-F_wrl(1)*vehicle_param.tr2-F_wrl(2)*vehicle_param.b+F_wrr(1)*vehicle_param.tr2-F_wrr(2)*vehicle_param.b+(F_wfl(1)*sin(steer_angle)+F_wfl(2)*cos(steer_angle)+F_wfr(1)*sin(steer_angle)+F_wfr(2)*cos(steer_angle))*vehicle_param.a+(-F_wfl(1)*cos(steer_angle)+F_wfl(2)*sin(steer_angle)+F_wfr(1)*cos(steer_angle)-F_wfr(2)*sin(steer_angle))*vehicle_param.tf2)/vehicle_param.Izz;
dX(vehicle_param.n_dofs+4)=(inputs.T(1,k)-F_wfl(1)*vehicle_param.r_w)/vehicle_param.I_w;
dX(vehicle_param.n_dofs+5)=(inputs.T(2,k)-F_wfr(1)*vehicle_param.r_w)/vehicle_param.I_w;
dX(vehicle_param.n_dofs+6)=(inputs.T(3,k)-F_wrl(1)*vehicle_param.r_w)/vehicle_param.I_w;
dX(vehicle_param.n_dofs+7)=(inputs.T(4,k)-F_wrr(1)*vehicle_param.r_w)/vehicle_param.I_w;

[a_car]=rot_global_to_car*transpose([dX(vehicle_param.n_dofs+1),dX(vehicle_param.n_dofs+2)]);





% Additional interesting data can be stored in data:
data.F_wfllong=F_wfl(1);
data.F_wfrlong=F_wfr(1);
data.F_wrllong=F_wrl(1);
data.F_wrrlong=F_wrr(1);
data.F_wfllat=F_wfl(2);
data.F_wfrlat=F_wfr(2);
data.F_wrllat=F_wrl(2);
data.F_wrrlat=F_wrr(2);
data.a_long=a_car(1);
data.a_lat=a_car(2);
% data.dX=dX;
% data.T=inputs.T;


