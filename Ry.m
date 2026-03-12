function [T,R,P,R_dq] = Ry(alpha)
% This function computes the rotation HTM y axis. Also it computes the
% partial derivative of R matrix

T = [cos(alpha) 0 sin(alpha) 0; 0 1 0 0; -sin(alpha) 0 cos(alpha) 0; 0 0 0 1;];
R = T(1:3,1:3);
P = T(1:3,4);
R_dq = [-sin(alpha) 0 cos(alpha); 0 0 0; -cos(alpha) 0 -sin(alpha)];
end