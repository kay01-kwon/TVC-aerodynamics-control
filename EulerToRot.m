function RotM = EulerToRot(phi,theta)
% EULERTOROT Summary of this function goes here
%   Detailed explanation goes here

ctheta = cos(theta);
stheta = sin(theta);
cphi = cos(phi);
sphi = sin(phi);

RotM = [ctheta  stheta*sphi stheta*cphi;
        0       cphi        -sphi;
        -stheta ctheta*sphi ctheta*cphi];
end