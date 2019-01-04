function inputs = set_inputs(vehicle_param)
%
% This function is for defining the inputs for the time-simulation in a structure
% 'inputs'. This always has to include the time-vector and the time-step.  
%

% Obligatory time information:
inputs.dt = 0.001;
inputs.time = (0:inputs.dt:6);

inputs.V0=15;

% Other inputs are dependent on the model used. (eg. steering angle, engine
% torque, ...)

% Wheel Torque:
inputs.T = ones(4, length(inputs.time));
inputs.T(1,:) = inputs.T(1,:)*0;
inputs.T(2,:) = inputs.T(2,:)*0;
% inputs.T(3,:) = inputs.T(3,:)*100;
% inputs.T(4,:) = inputs.T(4,:)*100;
% 
inputs.T(3,:) = inputs.T(3,:)*100+GetTVSignal(inputs.dt)*90; %crude torque vectoring
inputs.T(4,:) = inputs.T(4,:)*100+GetTVSignal(inputs.dt)*-90;

% Inclination of road:
inputs.inclin_angle = 0*pi/180; % [rad]

% Steering angle:
%inputs.delta = ones(size(inputs.time))*(5*pi/180);
inputs.delta = GetInputSignal(inputs.dt);

