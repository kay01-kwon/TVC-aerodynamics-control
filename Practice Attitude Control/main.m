clear all;
clc;
close all;

dt = 0.002;
cdt = 0.01;
Tf = 5;
time = 0;

s_init = [0;0];
param.z = 0.5;
param.Iyy = 1;

lambda = 2;

param.lambda1 = 2*lambda;
param.lambda2 = lambda^2;

T = 500;
Rad2Deg = 180/pi;
Deg2Rad = pi/180;
ref = -10*Deg2Rad;

u = 0;
u_saved = [];
time_saved = [];
while(time < Tf)

    t = time:dt:time+cdt;

    [time s] = ode45(@(t,s) attitude_dynamics(t,s,u,T,param),t,s_init);
    [s_model dsdt_model]= ref_model(s(end,:)',ref,param);
    u = control_law(T,s(end,:)',s_model,dsdt_model,ref,param);
    
%     ref = 10*time(end)*pi/180;
    if abs(ref) > 10*Deg2Rad
        ref = 10*sign(ref)*Deg2Rad;
    end
    
    subplot(2,1,1)
    plot(t,s(:,1)*Rad2Deg,'k')
    hold on;
    plot(time_saved,u_saved*Rad2Deg,'B')
    grid on;
    xlim([0 Tf])
    title('\theta (deg)- t')
    legend('\theta_{Body}','u_{\theta}')

    subplot(2,1,2)
    plot(t,s(:,2)*Rad2Deg,'k')
    hold on;
    grid on;
    xlim([0 Tf])
    title('d\theta/dt (deg/s)- t')
    pause(0.01)
    time = time + cdt;
    s_init = s(end,:);
    
    u_saved = [u_saved;u];
    time_saved = [time_saved;time(end)]; 
end