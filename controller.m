function [T,phi,theta] = controller(s,ref)
%CONTROLLER Summary of this function goes here
%   Detailed explanation goes here
global param;
m = param.m;
g = param.g;
CoG_r_T = param.CoG_r_T;
dx = -CoG_r_T(1);
dy = -CoG_r_T(2);
dz = -CoG_r_T(3);
dx = 0;
dy = 0;

% Heading and upward
x = [1;0;0];
z = [0;0;1];

p = s(4:6)';
v = s(1:3)';


% Control Input Bound
T_upper = 70*g;
T_lower = 50*g;

phi_eq = -asin(dy/sqrt(CoG_r_T'*CoG_r_T));
theta_eq = atan2(dx/cos(phi_eq),dz/cos(phi_eq));

qx = s(10); qy = s(11); qz = s(12); qw = s(13);
q = [qx;qy;qz;qw];

IRB = QuatToRot(q);

z_ = IRB*z
x_ = IRB*x

% Control Gain
Kp = [50 0 0;
    0 50 0;
    0 0 100];
Kd = [30 0 0;
    0 30 0;
    0 0 70];

% Expressed in the inertial frame
u = [0;0;-m*g] - Kp*(p - ref(1:3)) - Kd*(v - ref(4:6));

if abs(u(1)) > 100
    u(1) = sign(u(1))*100;
end

if abs(u(2)) > 100
    u(2) = sign(u(2))*100;
end

T = -sqrt(u'*u);

if abs(T) > T_upper
    T = -T_upper;
end

if abs(T) < T_lower
    T = -T_lower;
end

u

phi_s = asin(IRB(3,2));
phi_des = asin(u(2)/60/9.81) - phi_eq;

theta_s = atan2(-IRB(3,1)/cos(phi_s),IRB(3,3)/cos(phi_s));
theta_des =-asin(u(1)/60/9.81) - theta_eq;

phi = phi_eq + 0.2*(phi_des-phi_s) - 0.01*s(7);
theta = theta_eq + 0.5*(theta_des-theta_s) - 0.1*s(8);

threshold = 5;
threshold_rad = deg2rad(threshold);

if abs(phi) > threshold_rad
    phi = sign(phi)*threshold_rad;
end

if abs(theta) > threshold_rad
    theta = sign(theta)*threshold_rad;
end

fprintf("Phi des: %f (deg) \t",phi_des*180/pi);
fprintf("Theta des: %f (deg) \t",theta_des*180/pi);
fprintf("Phi state: %f (deg)\t",rad2deg(phi_s));
fprintf("Theta state: %f (deg)\n",rad2deg(theta_s));
fprintf("Phi eq: %f (deg)",rad2deg(phi_eq));
fprintf("Theta eq: %f (deg)\n",rad2deg(theta_eq));
fprintf("Control Input - Phi_u : (deg)%f Theta_u : %f (deg)",rad2deg(phi),rad2deg(theta));
fprintf("Thrust: %f \n",T);