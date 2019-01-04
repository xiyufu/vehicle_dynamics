function [delta, t] = GetInputSignal(dt)
%
% This function generates the steering angle (delta in radians) for the assignment. It
% also generates the time-vector (t in seconds), and the user can select the
% time-sampling-rate (dt in seconds) which is used.

t = (0:dt:6);

delta = zeros(1,length(t)); 

t1 = 0.5;
[~,i1] = min(abs(t-t1));
t2 = 1.3;
[~,i2] = min(abs(t-t2));
delta(i1:i2) = 1;


t1 = 1.8;
[~,i1] = min(abs(t-t1));
t2 = 4.5;
[~,i2] = min(abs(t-t2));
delta(i1:i2) = -1.5;


