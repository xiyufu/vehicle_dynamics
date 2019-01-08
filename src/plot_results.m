% post processing. plot results

% % compare input steering angle with yaw angle
% % normalize yaw
% yaw = wrapToPi(X(3,:));
% 
[ref, t] = GetInputSignal(inputs.dt);
% 
% figure;
% hold on
% plot(t, ref);
% plot(t, yaw);

% traction limits
F_wfllong = vertcat(data.F_wfllong);
F_wfllat = vertcat(data.F_wfllat);
F_wfrlong = vertcat(data.F_wfrlong);
F_wfrlat = vertcat(data.F_wfrlat);
F_wrllong = vertcat(data.F_wrllong);
F_wrllat = vertcat(data.F_wrllat);
F_wrrlong = vertcat(data.F_wrrlong);
F_wrrlat = vertcat(data.F_wrrlat);
F_nfl = vertcat(data.F_nfl);
F_nfr = vertcat(data.F_nfr);
F_nrl = vertcat(data.F_nrl);
F_nrr = vertcat(data.F_nrr);

F_wfl = sqrt(F_wfllong.^2 + F_wfllat.^2);
F_wfr = sqrt(F_wfrlong.^2 + F_wfrlat.^2);
F_wrl = sqrt(F_wrllong.^2 + F_wrllat.^2);
F_wrr = sqrt(F_wrrlong.^2 + F_wrrlat.^2);

figure
hold on
plot(t(2:end), F_wfl)
plot(t(2:end), F_nfl*vehicle_param.mu)
legend('vector sum of forces', 'friction limit');
title('friction limit and traction force, front left');
figure
hold on
plot(t(2:end), F_wfr)
plot(t(2:end), F_nfr*vehicle_param.mu);
legend('vector sum of forces', 'friction limit');
title('friction limit and traction force, front right');
figure
hold on
plot(t(2:end), F_wrl)
plot(t(2:end), F_nrl*vehicle_param.mu);
legend('vector sum of forces', 'friction limit');
title('friction limit and traction force, rear left');
figure
hold on
plot(t(2:end), F_wrr)
plot(t(2:end), F_nrr*vehicle_param.mu);
legend('vector sum of forces', 'friction limit');
title('friction limit and traction force, rear right');
% plot(t(2:end), F_wrl)
% plot(t(2:end), F_wrr)
% legend('fl','fr', 'rl', 'rr');