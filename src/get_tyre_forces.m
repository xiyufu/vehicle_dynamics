function [F_wfl, F_wfr, F_wrl, F_wrr, data] = get_tyre_forces(a_car_old,V_car,dphidt,steer_angle,F_aero,vehicle_param,inputs,X,k)

Vlong=V_car(1);
Vlat=V_car(2);
a_longold=a_car_old(1);
a_latold=a_car_old(2);

%% Calculate wheel velocities:

[Vwfl_long,Vwfl_lat,beta_fl] = get_wheel_velocities(Vlong,Vlat,dphidt,steer_angle,vehicle_param.a,vehicle_param.tf2);
[Vwfr_long,Vwfr_lat,beta_fr] = get_wheel_velocities(Vlong,Vlat,dphidt,steer_angle,vehicle_param.a,-vehicle_param.tf2);
[Vwrl_long,Vwrl_lat,beta_rl] = get_wheel_velocities(Vlong,Vlat,dphidt,0,-vehicle_param.b,vehicle_param.tr2);
[Vwrr_long,Vwrr_lat,beta_rr] = get_wheel_velocities(Vlong,Vlat,dphidt,0,-vehicle_param.b,-vehicle_param.tr2);

%% Calculate longitudinal slip:

omega_Rfl = X(vehicle_param.n_dofs+4)*vehicle_param.r_w;
omega_Rfr = X(vehicle_param.n_dofs+5)*vehicle_param.r_w;
omega_Rrl = X(vehicle_param.n_dofs+6)*vehicle_param.r_w;
omega_Rrr = X(vehicle_param.n_dofs+7)*vehicle_param.r_w;

[slip_fl, sign_Fxfl] = calc_slip(Vwfl_long, omega_Rfl);
[slip_fr, sign_Fxfr] = calc_slip(Vwfr_long, omega_Rfr);
[slip_rl, sign_Fxrl] = calc_slip(Vwrl_long, omega_Rrl);
[slip_rr, sign_Fxrr] = calc_slip(Vwrr_long, omega_Rrr);

%% Calculate normal forces:

F_nfaxle=(-F_aero*(vehicle_param.h_aero)-vehicle_param.M*a_longold*(vehicle_param.h)+vehicle_param.M*9.81*vehicle_param.b-inputs.T(1,k)*cos(steer_angle)-inputs.T(2,k)*cos(steer_angle)-inputs.T(3,k)-inputs.T(4,k))/vehicle_param.L;
F_nraxle=(F_aero*(vehicle_param.h_aero)+vehicle_param.M*a_longold*(vehicle_param.h)+vehicle_param.M*9.81*vehicle_param.a+inputs.T(1,k)*cos(steer_angle)+inputs.T(2,k)*cos(steer_angle)-inputs.T(3,k)-inputs.T(4,k))/vehicle_param.L;

F_nfl=(F_nfaxle*vehicle_param.tf2-a_latold*vehicle_param.M)/(2*vehicle_param.tf2);
F_nfr=(F_nfaxle*vehicle_param.tf2+a_latold*vehicle_param.M)/(2*vehicle_param.tf2);
F_nrl=(F_nraxle*vehicle_param.tr2-a_latold*vehicle_param.M)/(2*vehicle_param.tr2);
F_nrr=(F_nraxle*vehicle_param.tr2+a_latold*vehicle_param.M)/(2*vehicle_param.tr2);

%% Calculate tyre forces:

[F_wfl] = tyre_model_Dugoff(F_nfl, beta_fl, slip_fl, vehicle_param.mu, vehicle_param.Cx_f, vehicle_param.Cy_f, sign_Fxfl);
[F_wfr] = tyre_model_Dugoff(F_nfr, beta_fr, slip_fr, vehicle_param.mu, vehicle_param.Cx_f, vehicle_param.Cy_f, sign_Fxfr);
[F_wrl] = tyre_model_Dugoff(F_nrl, beta_rl, slip_rl, vehicle_param.mu, vehicle_param.Cx_r, vehicle_param.Cy_r, sign_Fxrl);
[F_wrr] = tyre_model_Dugoff(F_nrr, beta_rr, slip_rr, vehicle_param.mu, vehicle_param.Cx_r, vehicle_param.Cy_r, sign_Fxrr);

% Additional interesting data can be stored in data:
% data.Vwfl_long=Vwfl_long;
% data.slip_fl=slip_fl;
% data.slip_fr=slip_fr;
% data.slip_rl=slip_rl;
% data.slip_rr=slip_rr;
data.F_nfl=F_nfl;
data.F_nfr=F_nfr;
data.F_nrl=F_nrl;
data.F_nrr=F_nrr;
data.beta_fl=beta_fl;
data.beta_fr=beta_fr;
data.beta_rl=beta_rl;
data.beta_rr=beta_rr;
