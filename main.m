clear all;
close all;
clc

% Parameter set up
global param;

dx = -0.05;
dy = 0.05;
dz = -0.7;

param.m = 40;
param.Ixx = 1;
param.Iyy = 1;
param.Izz = 1;
param.g = 9.81;
param.CoG_r_T = [dx;dy;dz];

% Time 

dt = 0.01;
cdt = 0.05;
T = 30;
time = 0;

% State initialization
s_init = zeros(13,1);
s_init(13) = 1;

state_save = [];
time_save = [];

T_input = -param.m*param.g;
phi = -asin(-dy/sqrt(param.CoG_r_T'*param.CoG_r_T));
theta = atan2(-dx/cos(phi),-dz/cos(phi));

ref = [0;0;-1;0;0;0];

% figure('units','normalized','outerposition',[0 0 1 1])

while(time < T)

    t = time:dt:time+cdt;

    
    [time s] = ode45(@(t,s) plant_dynamics(t,s,T_input,phi,theta)',t,s_init);
    [T_input,phi,theta] = controller(s(end,:),ref);
    subplot(2,3,1);
    plot(time,s(:,4),'k')
    title('x - t')
    hold on;
    grid on;
    xlim([0 T])
    
    subplot(2,3,2);
    plot(time,s(:,5),'k')
    title('y - t')
    hold on
    grid on;
    xlim([0 T])

    subplot(2,3,3);
    plot(time,-s(:,6),'k')
    title('z - t')
    hold on
    grid on;
    xlim([0 T])

    subplot(2,3,4);
    plot(time,s(:,1),'k')
    title('v_x - t')
    hold on;
    grid on;
    xlim([0 T])
    
    subplot(2,3,5);
    plot(time,s(:,2),'k')
    title('v_y - t')
    hold on
    grid on;
    xlim([0 T])

    subplot(2,3,6);
    plot(time,-s(:,3),'k')
    title('v_z - t')
    hold on
    grid on;
    xlim([0 T])
    
    pause(0.0001)
    time = time + cdt;
    s_init = s(end,:)';
    
    s(end,7:9);

    time_save = [time_save; time];
    state_save = [state_save;s];

end