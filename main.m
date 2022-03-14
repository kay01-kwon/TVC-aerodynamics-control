clear all;
close all;
clc

% Parameter set up
global param;

dx = -0.020;
dy = -0.010;
dz = -(1733-794)/1000;
% dz = -0.572;
rpm = 0;
rpm2radpsec = 60/2/pi;

param.m = 60;
param.Ixx = 7;
param.Iyy = 12;
param.Izz = 1;
param.g = 9.81;
param.CoG_r_T = [dx;dy;dz];
param.L = 0.0322*rpm*rpm2radpsec;


% Time
dt = 0.01;
cdt = 0.05;
T = 30;
time = 0;

% State initialization
s_init = zeros(13,1);
ypr = [0 0 0];
ypr = ypr * pi/180;
quat = eul2quat(ypr);
s_init(10:13) = [quat(2);quat(3);quat(4);quat(1)];


state_save = [];
time_save = [];
control_time_save = [];

phi_u_save = [];
theta_u_save = [];

theta_s_save = [];
phi_s_save = [];

T_input = -param.m*param.g;
phi = -asin(-dy/sqrt(param.CoG_r_T'*param.CoG_r_T));
theta = atan2(-dx/cos(phi),-dz/cos(phi));

ref = [0;0;-1;0;0;0];


while(time < T)

    t = time:dt:time+cdt;

    
    [time s] = ode45(@(t,s) plant_dynamics(t,s,T_input,phi,theta)',t,s_init);
    [T_input,phi,theta] = controller(s(end,:),ref);
    
    qx = s(end,10);
    qy = s(end,11);
    qz = s(end,12);
    qw = s(end,13);
    
    q = [qx;qy;qz;qw];
    
    IRB = QuatToRot(q);
    phi_s = asin(IRB(3,2));
    theta_s = atan2(-IRB(3,1)/cos(phi_s),IRB(3,3)/cos(phi_s));
    
    subplot(3,3,1);
    plot(time,s(:,4),'k')
    title('x - t')
    hold on;
    grid on;
    xlim([0 T])
    
    subplot(3,3,2);
    plot(time,s(:,5),'k')
    title('y - t')
    hold on
    grid on;
    xlim([0 T])

    subplot(3,3,3);
    plot(time,-s(:,6),'k')
    title('z - t')
    hold on
    grid on;
    xlim([0 T])

    subplot(3,3,4);
    plot(time,s(:,1),'k')
    title('v_x - t')
    hold on;
    grid on;
    xlim([0 T])
    
    subplot(3,3,5);
    plot(time,s(:,2),'k')
    title('v_y - t')
    hold on
    grid on;
    xlim([0 T])

    subplot(3,3,6);
    plot(time,-s(:,3),'k')
    title('v_z - t')
    hold on
    grid on;
    xlim([0 T])
    
    subplot(3,3,7);
    plot(time,s(:,7),'k')
    title('\omega_x - t')
    hold on
    grid on;
    xlim([0 T])
    
    subplot(3,3,8);
    plot(time,s(:,8),'k')
    title('\omega_x - t')
    hold on
    grid on;
    xlim([0 T])
    
    subplot(3,3,9);
    plot(time,s(:,9),'k')
    title('\omega_x - t')
    hold on
    grid on;
    xlim([0 T])
    
    pause(0.0001)
    time = time + cdt;
    s_init = s(end,:)';
    
    s(end,7:9);

    time_save = [time_save; time];
    state_save = [state_save;s];
    control_time_save = [control_time_save;time(end)];
    
    phi_u_save = [phi_u_save; phi];
    theta_u_save = [theta_u_save; theta];
    
    phi_s_save = [phi_s_save; phi_s];
    theta_s_save = [theta_s_save;theta_s];

end

%%%
figure()
subplot(2,2,1)
plot(control_time_save,rad2deg(phi_u_save))
title('\phi_{u} (deg) - time (s)')
xlabel('time (s)')
ylabel('\phi_{u} (deg)')
grid on;

subplot(2,2,2)
plot(control_time_save,rad2deg(theta_u_save))
title('\theta_{u} (deg) - time (s)')
xlabel('time (s)')
ylabel('\theta_{u} (deg)')
grid on;

subplot(2,2,3)
plot(control_time_save,rad2deg(phi_s_save))
title('\phi_{s} (deg) - time (s)')
xlabel('time (s)')
ylabel('\phi_{s} (deg)')
grid on;

subplot(2,2,4)
plot(control_time_save,rad2deg(theta_s_save))
title('\theta_{s} (deg) - time (s)')
xlabel('time (s)')
ylabel('\theta_{s} (deg)')
grid on;
