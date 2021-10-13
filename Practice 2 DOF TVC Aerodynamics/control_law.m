function u_theta = control_law(T,s,s_model,dsdt_model,ref,param)
%CONTROL_LAW Summary of this function goes here
%   Detailed explanation goes here
z = param.z;
Iyy = param.Iyy;
lambda1 = param.lambda1;
lambda2 = param.lambda2;


u_theta = Iyy/z/T*(dsdt_model(2) - lambda1*(s(2)-s_model(2)) - lambda2*(s(1)-s_model(1)));

end

