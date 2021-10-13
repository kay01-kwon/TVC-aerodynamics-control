function [s_curr dsdt] = ref_model(s_prev,ref,param)
%REF_MODEL Summary of this function goes here
%   Detailed explanation goes here
lambda1 = param.lambda1;
lambda2 = param.lambda2;
dt = 0.01;

A = [0          1;
    -lambda2   -lambda1];

B = [0;lambda2];

dsdt = A*s_prev + B*ref;

s_curr = s_prev + dsdt*dt;

end

