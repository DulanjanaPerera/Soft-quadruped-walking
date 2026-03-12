function [T,R,P,R_dq] = Rz(alpha)
% This function computes the rotation HTM z axis. Also it computes the
% partial derivative of R matrix


T = [cos(alpha) -sin(alpha) 0 0; sin(alpha) cos(alpha) 0 0; 0 0 1 0; 0 0 0 1;];
R = T(1:3,1:3);
P = T(1:3,4);
R_dq = [-sin(alpha) -cos(alpha) 0; cos(alpha) -sin(alpha) 0; 0 0 0];
end