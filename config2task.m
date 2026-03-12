function p = config2task(theta, phi, L)
% THis function computes the length changes forgiven theta and phi.
% 
% Inputs:
%   theta   : Bending direction [constant or vector] (rad)
%   phi     : bending amount [constant or vector] (rad)
%   L       : Initial length of the PMA [constant] (m)
%
% Output:
%   p       : position of the tip [3x1] (m)

len = length(theta);
p = zeros(3, len);

phi(abs(phi)<=1e-5) = 0.001;

if phi <= 1e-5    
    p(3,:) = L;
else
    % Compute length changes for non-negligible phi
    p(1,:) = (L./phi) .* (1 - cos(phi)) .* cos(theta);
    p(2,:) = (L./phi) .* (1 - cos(phi)) .* sin(theta);
    p(3,:) = (L./phi) .* sin(phi);

end

end