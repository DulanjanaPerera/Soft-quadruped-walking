function [T, R, P] = HTM_nume(l, xi, L, r)
%#codegen
% Compute the Homogeneous Transforamtion Matrix for a section. The symbolic equation is computed from the 
% Maple file.
% 
% The derivations are referenced from following paper.
%   * Godage, Isuru S., Robert J. Webster, and Ian D. Walker. "Center-of-gravity-based 
%     approach for modeling dynamics of multisection continuum arms." IEEE transactions 
%     on robotics 35, no. 5 (2019): 1097-1108.
% 
% Inputs:
%   l   : length change of the PMA [3x1] [constant] (m)
%   xi  : selection factor of the backbone. xi=0 is the base and xi=1 is
%         the tip. [constant] {0,1}
%   L   : Length of the continuum arm [constant] (m)
%   r   : radial offset of the PMA [constant] (m)
% 
% Outputs:
%   T   : HTM of the continuum arm at xi [4x4]
%   R   : Rotation matrix [3x3]
%   P   : position vector [3x1]

arguments (Input)
    l (1,3) double 
    xi (1,1) double {mustBeBetween(xi, 0, 1, "closed")} = 1
    L (1,1) double {mustBeGreaterThanOrEqual(L, 0)} = 0.278
    r (1,1) double {mustBePositive} = 0.013
end

arguments (Output)
    T (4,4) double
    R (3,3) double
    P (3,1) double
end

T = [(l(1) + l(2)) * (0.2e1 / 0.1148175e7 * (l(1) ^ 2 + l(1) * l(2) + l(2) ^ 2) ^ 4 * xi ^ 8 - r ^ 2 * (l(1) ^ 2 + l(1) * l(2) + l(2) ^ 2) ^ 3 * xi ^ 6 / 0.8505e4 + 0.2e1 / 0.405e3 * r ^ 4 * (l(1) ^ 2 + l(1) * l(2) + l(2) ^ 2) ^ 2 * xi ^ 4 - r ^ 6 * (l(1) ^ 2 + l(1) * l(2) + l(2) ^ 2) * xi ^ 2 / 0.9e1 + r ^ 8) * xi ^ 2 * L / r ^ 9 / 0.2e1 -(-l(2) + l(1)) * xi ^ 2 * (0.2e1 / 0.1148175e7 * (l(1) ^ 2 + l(1) * l(2) + l(2) ^ 2) ^ 4 * xi ^ 8 - r ^ 2 * (l(1) ^ 2 + l(1) * l(2) + l(2) ^ 2) ^ 3 * xi ^ 6 / 0.8505e4 + 0.2e1 / 0.405e3 * r ^ 4 * (l(1) ^ 2 + l(1) * l(2) + l(2) ^ 2) ^ 2 * xi ^ 4 - r ^ 6 * (l(1) ^ 2 + l(1) * l(2) + l(2) ^ 2) * xi ^ 2 / 0.9e1 + r ^ 8) * L * sqrt(0.3e1) / r ^ 9 / 0.6e1 (r ^ 10 - 0.2e1 / 0.9e1 * xi ^ 2 * (l(1) ^ 2 + l(1) * l(2) + l(2) ^ 2) * r ^ 8 + 0.2e1 / 0.135e3 * xi ^ 4 * (l(1) ^ 2 + l(1) * l(2) + l(2) ^ 2) ^ 2 * r ^ 6 - 0.4e1 / 0.8505e4 * xi ^ 6 * (l(1) ^ 2 + l(1) * l(2) + l(2) ^ 2) ^ 3 * r ^ 4 + 0.2e1 / 0.229635e6 * xi ^ 8 * (l(1) ^ 2 + l(1) * l(2) + l(2) ^ 2) ^ 4 * r ^ 2 - 0.4e1 / 0.37889775e8 * xi ^ 10 * (l(1) ^ 2 + l(1) * l(2) + l(2) ^ 2) ^ 5) * xi * L / r ^ 10];

R = T(1:3,1:3);
P = T(1:3,4);

end