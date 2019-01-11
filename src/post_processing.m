function [] = post_processing(X, data, vehicle_param, inputs)
%
% This function is for post_processing the information form the
% time-integration. 

% 
% figure
% subplot(311)
% plot(inputs.time, X(1,:)), axis tight, xlabel('time [s]'), ylabel('X(1) [?]')
% subplot(312)
% plot(inputs.time, X(2,:)), axis tight, xlabel('time [s]'), ylabel('X(1) [?]')
% subplot(313)
% plot(inputs.time, X(3,:)), axis tight, xlabel('time [s]'), ylabel('X(1) [?]')


%%
for k = 1:20:length(inputs.time)
    
   
    p(:,1) = [X(1,k) - vehicle_param.b*cos(X(3,k)); X(2,k) - vehicle_param.b*sin(X(3,k))];
    p(:,2) = [X(1,k) + vehicle_param.a*cos(X(3,k)); X(2,k) + vehicle_param.a*sin(X(3,k))];
%     p(:,3) = [X(1,k) - vehicle_param.b*cos(X(3,k)); X(2,k) - vehicle_param.b*sin(X(3,k))];
%     p(:,4) = [X(1,k) + vehicle_param.a*cos(X(3,k)); X(2,k) + vehicle_param.a*sin(X(3,k))];
    
    figure(123)
    plot(p(1,:)', p(2,:)', 'b-o', 'linewidth', 2)
    if max(X(1,:))>max(X(2,:))
         axis([min(X(1,:)) max(X(1,:)) min(X(1,:)) max(X(1,:))])
     else
         axis([min(X(2,:)) max(X(2,:)) min(X(2,:)) max(X(2,:))])
     end
    
     pause(0.0005)
end

%% compare input steering angle with yaw angle
% normalize yaw
yaw = wrapToPi(X(3,:));

[ref, t] = GetInputSignal(inputs.dt, inputs.t_end);

figure;
hold on
grid on
plot(t, ref, 'LineWidth', 2);
plot(t, yaw, 'LineWidth', 2);
legend('Steering angle', 'Yaw angle');
%%

% energy = zeros(size(inputs.time));
% for k = 1:length(inputs.time)
%     
%     energy(k) = sqrt(X(vehicle_param.n_dofs+1,k)^2+X(vehicle_param.n_dofs+2,k)^2)*vehicle_param.M + X(vehicle_param.n_dofs+3,k)*vehicle_param.Izz + X(vehicle_param.n_dofs+4,k)*vehicle_param.I_w+X(vehicle_param.n_dofs+5,k)*vehicle_param.I_w;
%     
% end
% 
% figure
% plot(inputs.time, energy, 'linewidth', 1.5), axis tight, xlabel('t [s]'), ylabel('Energy [J]')