function dsdt = plant_dynamics(t,s,Thrust,phi,theta)
%PLANT_DYNAMICS Summary of this function goes here
%   Detailed explanation goes here

% s: state variables
% s(1:3) = v,   dsdt(1:3) = dvdt 
% s(4:6) = p,   dsdt(4:6) = dpdt
% s(7:9) = w,   dsdt(7:9) = dwdt
% s(10:13) = q, dsdt(10:13) = dqdt
 
global param;

m = param.m;
Ixx = param.Ixx;
Iyy = param.Iyy;
Izz = param.Izz;
I = diag([Ixx Iyy Izz]);
g = param.g;
CoG_r_T = param.CoG_r_T;
L_ = [0;0;param.L];

T = [0;0;Thrust];

% Translational dynamics and kinematics
% State variables - Position of Center of Gravity
q = s(10:13);
qx = q(1); qy = q(2); qz = q(3); qw = q(4);
q_v = [qx;qy;qz];
dsdt(1:3) = QuatToRot(q)*EulerToRot(phi,theta)*T/m + [0;0;g];
dsdt(4:6) = s(1:3);

% Rotational dynamics and kinematics
w = s(7:9);
dsdt(7:9) = I\(cross(CoG_r_T, EulerToRot(phi,theta)*T) - cross(w,I*w) + cross(w,L_));
dsdt(10:13) = 0.5*[cross(q_v,w) + qw*w;-q_v'*w;];

if abs(Thrust) < abs(m*g) && s(6) > 0
    dsdt(1:13) = zeros(1,13);
    s(1:12) = zeros(1,12);
    s(13) = 1;
end

end