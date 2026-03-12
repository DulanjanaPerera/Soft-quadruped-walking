function [T,R,P] = Tx(x)
% This function computes the HTM x axis

T = eye(4);
T(1,4) = x;
R = T(1:3,1:3);
P = T(1:3,4);
end