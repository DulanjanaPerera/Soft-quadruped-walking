function [T,R,P, R_dq] = Rx(alpha)
% This function computes the rotation HTM x axis. Also it computes the
% partial derivative of R matrix

T = [1 0 0 0; 0 cos(alpha) -sin(alpha) 0; 0 sin(alpha) cos(alpha) 0; 0 0 0 1];
R = T(1:3,1:3);
P = T(1:3,4);
R_dq = [0 0 0; 0 -sin(alpha) -cos(alpha); 0 cos(alpha) -sin(alpha)];
end