function RotM = QuatToRot(q)
% QUATTOROT Summary of this function goes here
%   Detailed explanation goes here
% q(1) = qx, q(2) = qy, q(3) = qz, q(4) = qw
qx = q(1); qy = q(2); qz = q(3); qw = q(4);

q_v = [qx;qy;qz];
q_v_skiew = [0   -qz  qy;
             qz   0   -qx;
             -qy  qx  0];
q_xyz = q_v*q_v';

RotM = qw^2*eye(3) + 2*qw*q_v_skiew + 2*q_xyz - q_v'*q_v*eye(3);
end