function dsdt = aerodynamics_plant(t,s,theta_input,T,param)
%AERODYNAMICS_PLANT Summary of this function goes here
%   Detailed explanation goes here
% s(1:2) = [vx;vz]
% s(3:4) = [x;z]
% s(5:6) = [w;theta]


z = param.z;
Iyy = param.Iyy;
m = param.m;
g = param.g;
% Translational dynamics
dsdt(1:2) = [-1/m*T*sin(theta_input + s(6));-1/m*T*cos(theta_input + s(6)) + g];

% Translational kinematics
dsdt(3:4) = s(1:2);

% Rotational dynamics
dsdt(5) = z/Iyy*T*sin(theta_input);
dsdt(6) = s(5);

end

