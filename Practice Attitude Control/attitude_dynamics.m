function dsdt = attitude_dynamics(t,s,u,T,param)
%ATTITUDE_DYNAMICS Summary of this function goes here
%   Detailed explanation goes here
% s(1): theta_b
% s(2): w_b

z = param.z;
Iyy = param.Iyy;

A = [0 1;
     0 0];

B = [0;z/Iyy*T];

dsdt = A*s + B*sin(u);
end

