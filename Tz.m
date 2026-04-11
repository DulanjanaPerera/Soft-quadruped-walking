function [T,R,P,R_dq] = Tz(x)
% This function computes the HTM z axis

T = eye(4);
T(3,4) = x;
R = T(1:3,1:3);
P = T(1:3,4);
R_dq = zeros(3,3);
end