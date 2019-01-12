function [dTfl, dTfr, dTrl, dTrr] = torque_vec(inputs, K, angles, Ffl, Ffr, Frl, Frr, vehicle_param, idx)
%torque_vec
%   Generating torque vectoring signals for a lane change
%   Change in the steering angle is used to generate assistant torques in
%   order to improve the transient performance during the lane change.
%   Torque is only apply to rear wheels.
%   The second part of this function is to make sure tyres are not
%   saturated.

delta_now = inputs.delta(idx);
time_window = 0.1; % compare current steering angle with the one 0.1s ago
idx_old = max(1, idx - time_window/inputs.dt);
delta_old = inputs.delta(idx_old);

dTfl = 0;
dTfr = 0; % RWD, front wheels have no inputs

T_desired = (inputs.T(3,1) + inputs.T(4,1))/2; 

% a positive steering angle is turning left
% so decrease left torque and increase right torque to get a higher yaw
% acceleration. K is a scaling parameter to be tuned.
dT = max(-T_desired, min((delta_now - delta_old)*K, T_desired)); % -200Nm <= dT <= 200 Nm
dTrl = -dT;
dTrr = dT;

% If the wheels are close to sliping, reduce the input torque.
% The first criterion is: slip angle can't be too large. Due to a finite
% acceleration, the velocity, hence the slip angle, has to change
% continuously. If the slip angle gets too large during cornering, it can't
% be reduced to a sufficiently low value before the second lane change. 
% The result is tyre saturation which can't be solved by reducing input 
% torques (Lateral force alone is enough to saturate the tyre).
% The second criterion is: reduce the input torque when we are close to
% saturation.

% Forces act on wheels
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

if abs(angles(3)) > 0.06 || sqrt(Fxrl^2+Fyrl^2) > vehicle_param.mu*Fzrl*0.75
    dTrl = -min(T_desired, max(T_desired*0.9, vehicle_param.mu*Fzrl*0.8));
end
if abs(angles(4)) > 0.06 || sqrt(Fxrr^2+Fyrr^2) > vehicle_param.mu*Fzrr*0.75
    dTrr = -min(T_desired, max(T_desired*0.9, vehicle_param.mu*Fzrl*0.8));
end

