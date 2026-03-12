function l = config2length(theta, phi, r)
% This function computes the Cartesian coordinate w.r.t. the base of the arm
% for given theta and phi.
% 
% Inputs:
%   theta   : Bending direction [constant or vector] (rad)
%   phi     : bending amount [constant or vector] (rad)
%   r       : radial offset of the PMA [constant] (m)
%
% Output:
%   l       : length changes [3x1] (m)

theta = theta(:).'; 
phi   = phi(:).'; 

len = length(theta);
l = 1e-6 .* ones(3, len);

phi(abs(phi)<=1e-5) = 0.001;

if phi <= 1e-5    
    return ;
else
    % Compute length changes for non-negligible phi
    l(1, :) = -r * cos(theta) .* phi;
    l(2, :) = (0.5 * r * cos(theta) - sqrt(3) * r * sin(theta) / 2) .* phi;
    l(3, :) = (0.5 * r * cos(theta) + sqrt(3) * r * sin(theta) / 2) .* phi;

end

end