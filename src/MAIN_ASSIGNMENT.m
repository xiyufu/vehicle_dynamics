
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main Vehicle Simulation %
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This is the main file for the simulation of a vehicle system. 
% In this file all parameters and inputs are defined, the integration
% algorithm is called and the results are post-processed. 

clear all

%% Define vehicle parameters:
vehicle_param = set_vehicle_param;


%% Define simulation inputs:
inputs = set_inputs(vehicle_param);


%% Perform time-integration of vehicle system over inputs:
[X, data] = time_integration(vehicle_param, inputs);


%% Post-process results: 
post_processing(X, data, vehicle_param, inputs);

