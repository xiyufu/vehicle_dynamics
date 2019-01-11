% post processing. plot results

% % compare input steering angle with yaw angle
% normalize yaw
yaw = wrapToPi(X(3,:));

[ref, t] = GetInputSignal(inputs.dt, inputs.t_end);

figure;
hold on
plot(t, ref);
plot(t, yaw);

% traction limits
figure;
hold on
title('friction limits');
% for k = 3:3:12
%     plot(t(2:end), vehicle_param.mu*data(k,:));
% end
% for k = 1:3:12
%     plot(t(2:end), sqrt(data(k,:).^2+data(k+1,:).^2));
% end
% legend('lim-fl','lim-fr','lim-rl','lim-rr', 'trac-fl', 'trac-fr', 'trac-rl', 'trac-rr');
plot(t(2:end), vehicle_param.mu*data(12,:)); % friction limit - rear right wheel
plot(t(2:end), sqrt(data(10,:).^2+data(11,:).^2)); % total traction - rear right wheel
plot(t(2:end), data(22,:)); % input torque - rear right wheel
plot(t(2:end), data(10,:)); % longitudinal force - rear right
plot(t(2:end), data(11, :)); %lateral force - rear right
plot(t, 1000*ref);
scatter(t(2:end), 1000*data(20,:),'x') % slip angles - rear right
legend('lim-rr', 'trac-rr', 'T-rr', 'flong-rr', 'flat-rr','steer', 'slip angle');

% rear left
figure;
hold on
title('friction limits');
plot(t(2:end), vehicle_param.mu*data(9,:)); % friction limit 
plot(t(2:end), sqrt(data(7,:).^2+data(8,:).^2)); % total traction 
plot(t(2:end), data(21,:)); % input torque 
plot(t(2:end), data(7,:)); % longitudinal force 
plot(t(2:end), data(8, :)); %lateral force
plot(t, ref*1000);
scatter(t(2:end), 1000*data(19,:), 'x'); % slip angles 
legend('lim-rl', 'trac-rl', 'T-rl', 'flong-rl', 'flat-rl','steer', 'slip angle');