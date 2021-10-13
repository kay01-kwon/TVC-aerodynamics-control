function [u_T theta_des] = PD_controller(r_des,s,param)
%PD_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here
g = param.g;
m = param.m;
Kp = 100*eye(2);
Kd = 70*eye(2);

e = s(3:4)'- r_des;
dedt = s(1:2)'-[0;0];

u =  Kp*e + Kd*dedt + m*[0;g];

u_T = sqrt(u'*u);
theta_des = atan2(u(1),u(2))

if abs(theta_des) > 10*pi/180
    theta_des = sign(theta_des)*10*pi/180;
end

end