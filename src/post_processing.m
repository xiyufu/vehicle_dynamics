function [] = post_processing(X, data, vehicle_param, inputs)
%
% This function is for post_processing the information form the
% time-integration. 

% 
% figure
% subplot(311)
% plot(inputs.time, X(1,:)), axis tight, xlabel('time [s]'), ylabel('X(1) [X]')
% subplot(312)
% plot(inputs.time, X(2,:)), axis tight, xlabel('time [s]'), ylabel('X(1) [Y]')
% subplot(313)
% plot(inputs.time, X(3,:)), axis tight, xlabel('time [s]'), ylabel('X(1) [phi]')

a_lat=vertcat(data.a_lat);

F_nfl=sum(vertcat(data.F_nfl))/length(vertcat(data.F_nfl));
F_nfr=sum(vertcat(data.F_nfr))/length(vertcat(data.F_nfr));
F_nrl=sum(vertcat(data.F_nrl))/length(vertcat(data.F_nrl));
F_nrr=sum(vertcat(data.F_nrr))/length(vertcat(data.F_nrr));
eta=(F_nfl+F_nfr)/vehicle_param.Cy_f - (F_nrl+F_nrr)/vehicle_param.Cy_r;
beta_diff_olley=(eta/9.81)*a_lat(200:end)*180/pi;

%figure(1)
%plot(a_lat(200:end),beta_diff_olley)
xlabel('Lateral Acceleration [m/s^2]'), ylabel('\beta_f - \beta_r [°]'), title ('Figure 1: Linear Tyre'),axis auto, grid on

beta_fl=abs(vertcat(data.beta_fl));
beta_rl=abs(vertcat(data.beta_rl));
beta_fr=abs(vertcat(data.beta_fr));
beta_rr=abs(vertcat(data.beta_rr));
%figure(2)
%plot(a_lat(200:end),[(beta_fl(200:end)-beta_rl(200:end))*(180/pi),(beta_fr(200:end)-beta_rr(200:end))*(180/pi)])
%xlabel('Lateral Acceleration [m/s^2]'), ylabel('\beta_f - \beta_r [°]'),title ('Figure 2: Simulation with a Dugoff Tyre Model'),axis auto, grid on, legend ('Left two wheels','Right two wheels')
%plot(data.a_lat,data.beta_f-data.beta_f), axis auto
%plot(inputs.time(1:end-1),vertcat(data.a_lat))
% figure(3)
% plot(inputs.time(1:end-1),vertcat(data.beta_fl)*180/pi,inputs.time(1:end-1),vertcat(data.beta_rl)*180/pi), legend ('beta_fl','beta_rl')

%
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
%%
% 
% energy = zeros(size(inputs.time));
% for k = 1:length(inputs.time)
%     
%     energy(k) = sqrt(X(vehicle_param.n_dofs+1,k)^2+X(vehicle_param.n_dofs+2,k)^2)*vehicle_param.M + X(vehicle_param.n_dofs+3,k)*vehicle_param.Izz + X(vehicle_param.n_dofs+4,k)*vehicle_param.I_w+X(vehicle_param.n_dofs+5,k)*vehicle_param.I_w;
%     
% end
% 
% figure
% plot(inputs.time, energy, 'linewidth', 1.5), axis tight, xlabel('t [s]'), ylabel('Energy [J]')