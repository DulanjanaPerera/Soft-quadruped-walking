function l = config2length(theta, phi, r)
% THis function computes the length changes forgiven theta and phi.
% 
% Inputs:
%   theta   : Bending direction [constant or vector] (rad)
%   phi     : bending amount [constant or vector] (rad)
%   r       : radial offset of the PMA [constant] (m)
%
% Output:
%   l       : length changes [3x1] (m)

l = zeros(3, 1);
if phi <= 1e-5    
    return ;
else
    % Compute length changes for non-negligible phi
    l = zeros(3, 1);
    l(1) = -r * cos(theta) * phi;
    l(2) = (0.5 * r * cos(theta) - sqrt(3) * r * sin(theta) / 2) * phi;
    l(3) = (0.5 * r * cos(theta) + sqrt(3) * r * sin(theta) / 2) * phi;

end

end