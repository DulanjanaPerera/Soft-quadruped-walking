function T = globalBodyHTM(B)
% The function compute the robot's gloabl position when gloabl parameters
% are given. The the robot's body frame is at the front junction.
% 
% Implementation is inspired from:
%   * Godage, Isuru S., Thrishantha Nanayakkara, and Darwin G. Caldwell. 
%     "Locomotion with continuum limbs." In 2012 IEEE/RSJ International 
%     Conference on Intelligent Robots and Systems, pp. 293-298. IEEE, 2012.
%
% Inputs:
%   B   : the vector of the gloabl spatial variarbles [6x1]
%         [x, y, z, x_rad, y_rad, z_rad]
%
% Outpu:
%   T   : The HTM of the robot's body frame w.r.t. the gloabl location



% Validate input dimensions
if length(B) ~= 6
    error('Input B must be a 6-element vector representing the robot parameters.');
end

T = [cos(B(5)) * cos(B(6)) -cos(B(5)) * sin(B(6)) sin(B(5)) B(1); sin(B(4)) * sin(B(5)) * cos(B(6)) + cos(B(4)) * sin(B(6)) -sin(B(4)) * sin(B(5)) * sin(B(6)) + cos(B(4)) * cos(B(6)) -sin(B(4)) * cos(B(5)) B(2); -cos(B(4)) * sin(B(5)) * cos(B(6)) + sin(B(4)) * sin(B(6)) cos(B(4)) * sin(B(5)) * sin(B(6)) + sin(B(4)) * cos(B(6)) cos(B(4)) * cos(B(5)) B(3); 0 0 0 1;];

end