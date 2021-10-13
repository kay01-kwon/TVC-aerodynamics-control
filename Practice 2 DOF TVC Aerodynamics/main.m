clear all;
clc;
close all;

dt = 0.005;
cdt = 0.01;
Tf = 10;
time = 0;

s_init = zeros(6,1);
param.z = 1.0;
param.Iyy = 1;
param.m = 50;
param.g = 9.81;

lambda = 10;

param.lambda1 = 2*lambda;
param.lambda2 = lambda^2;

Rad2Deg = 180/pi;
Deg2Rad = pi/180;

u_theta = 0;
u_T = 50*9.81;
while(time < Tf)

    t = time:dt:time+cdt;

    [time s] = ode45(@(t,s) aerodynamics_plant(t,s,u_theta,u_T,param)',t,s_init);
    
    [u_T theta_body_des] = PD_controller([1;-2],s(end,:),param);
    [s_model dsdt_model]= ref_model(s(end,5:6)',theta_body_des,param);
    u_theta = control_law(u_T,[s(end,6);s(end,5)],s_model,dsdt_model,theta_body_des,param);    
    
    
    subplot(2,2,1)
    plot(t,s(:,3),'k')
    hold on;
    grid on;
    xlim([0 Tf])
    title('x- t')

    subplot(2,2,2)
    plot(t,s(:,4),'k')
    hold on;
    grid on;
    xlim([0 Tf])
    title('z- t')
    
    subplot(2,2,3)
    plot(t,s(:,6)*Rad2Deg,'k')
    hold on;
    grid on;
    xlim([0 Tf])
    title('\theta- t')
    
    subplot(2,2,4)
    plot(t,s(:,5)*Rad2Deg,'k')
    hold on;
    grid on;
    xlim([0 Tf])
    title('d\theta/dt- t')
    
    pause(0.01)
    time = time + cdt;
    s_init = s(end,:)';

end