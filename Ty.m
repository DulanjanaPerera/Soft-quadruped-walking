function [T,R,P,R_dq] = Ty(x)
% This function computes the HTM y axis

T = eye(4);
T(2,4) = x;
R = T(1:3,1:3);
P = T(1:3,4);
R_dq = zeros(3,3);
end