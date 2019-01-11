function [X, data] = time_integration(vehicle_param, inputs)
%
% [X, data] = time_integration(vehicle_param, inputs)
%
%   vehicle_param = parameters of simulated vehicle system
%   inputs = inputs for simulation
%
%   X = state variables (positions and velocities)
%   data = structure with interesting data for post-processing (eg.
%   slip-angles, forces)
%   
% This function performs the time integration of the vehicle system for the
% given inputs with a FORWARD EULER INTEGRATION. 

% Initialize state-space vector:
v0 = inputs.v0;
X = zeros(vehicle_param.n_dofs*2, length(inputs.time));
X(vehicle_param.n_dofs+1) = v0;
X(vehicle_param.n_dofs+4) = v0/vehicle_param.r_w;
X(vehicle_param.n_dofs+5) = v0/vehicle_param.r_w;
X(vehicle_param.n_dofs+6) = v0/vehicle_param.r_w;
X(vehicle_param.n_dofs+7) = v0/vehicle_param.r_w;
dX = zeros(vehicle_param.n_dofs*2, length(inputs.time));

% Create waitbar to track simulation progress:
wbar = waitbar(0,'Simulation running. Please wait...');

% Loop over time for integration:
% data = zeros(12, length(inputs.time)-1);
for k = 1:length(inputs.time)-1
    
    % Calculate change of variables from equations of motion:
    [dX(:,k+1), data(:,k)] = equations_of_motion(vehicle_param, inputs, X(:,k), dX(:,k), k);
    
    % Euler integration step:
    X(:,k+1) = X(:,k) + dX(:,k+1)*inputs.dt;
    
    % Update wait-bar:
    waitbar(k/length(inputs.time), wbar)

end
% Close waitbar:
close(wbar)