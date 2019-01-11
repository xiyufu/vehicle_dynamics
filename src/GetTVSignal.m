function [delta, t] = GetTVSignal(dt)
%
% This function generates the steering angle (delta in radians) for the assignment. It
% also generates the time-vector (t in seconds), and the user can select the
% time-sampling-rate (dt in seconds) which is used.

t = (0:dt:6);

delta = zeros(1,length(t)); 

t1 = 0.5;
[~,i1] = min(abs(t-t1));
t2 = 1.0;
[~,i2] = min(abs(t-t2));
delta(i1:i2) = 0.8;


t1 = 2.5;
[~,i1] = min(abs(t-t1));
t2 = 3.5;
[~,i2] = min(abs(t-t2));
delta(i1:i2) = -1.5;


