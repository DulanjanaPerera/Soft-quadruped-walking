function [T,R,P] = Ty(x)
% This function computes the HTM y axis

T = eye(4);
T(2,4) = x;
R = T(1:3,1:3);
P = T(1:3,4);
end