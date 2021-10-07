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

% Heading and upward
x = [1;0;0];
z = [0;0;1];

p = s(4:6)';
v = s(1:3)';

% Control Input Bound
T_upper = 50*g;
phi_eq = -asin(dy/sqrt(CoG_r_T'*CoG_r_T));
theta_eq = atan2(dx/cos(phi_eq),dz/cos(phi_eq));

qx = s(10); qy = s(11); qz = s(12); qw = s(13);
q = [qx;qy;qz;qw];

IRB = QuatToRot(q);

z_ = IRB*z
x_ = IRB*x

% Control Gain
Kp = 100*eye(3);
Kd = 70*eye(3);

% Expressed in the inertial frame
p(1) = 0; p(2) = 0; v(1) = 0; v(2) = 0;
u = [0;0;-m*g] - Kp*(p - ref(1:3)) - Kd*(v - ref(4:6));
T = -sqrt(u'*u);

    if abs(T) > T_upper
        T = -T_upper;
    end    
phi_s = asin(IRB(3,2));
% phi_s = -asin(z_(2))
phi = phi_eq + 0.5*( -phi_s - phi_eq) - 0.1*s(7)
phi_eq_deg = phi_eq*180/pi

theta_s = atan2(-IRB(3,1)/cos(phi_s),IRB(3,3)/cos(phi_s));
% theta_s = atan2(z_(1)/cos(phi_s),z_(3)/cos(phi_s))
theta = theta_eq + 0.5*( -theta_s - theta_eq ) - 0.1*s(8)

theta_eq_deg = theta_eq*180/pi

if abs(phi) > 20*pi/180
    phi = sign(phi)*10*pi/180;
end

if abs(theta) > 20*pi/180
    theta = sign(theta)*10*pi/180;
end