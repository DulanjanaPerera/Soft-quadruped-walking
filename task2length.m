function [l,z] = task2length(p, r, L)
% Calculate the final position based on theta, phi, and length L
% 
% Inputs:
%   p   : task sapce variable. The tip position [1x2]
%   r   : radial offset of the PMA from the backbone [constant]
%   L   : backbone lenght [contant]
% 
% Output:
%   l   : actual space variable. Length changes [3x1]
%   z   : the z Cartisian coordinate


    if nargin < 2
        L = 0.278;
        r = 0.013;
    end

    l = zeros(1,3);

    theta = atan2(p(2), p(1));

    % fun = @(x)sqrt(((-cos(theta) * cos(x) * L / x + cos(theta) * L / x)-p(1))^2 +...
    %     ((-sin(theta) * cos(x) * L / x + sin(theta) * L / x)-p(2))^2 +...
    %     ((sin(x) * L / x)-p(3))^2);
    
    fun = @(x)sqrt(((-cos(theta) * cos(x) * L / x + cos(theta) * L / x)-p(1))^2 ...
                     + ((-sin(theta) * cos(x) * L / x + sin(theta) * L / x)-p(2))^2);

    % phi bounds are (0, pi]
    lb = 0.0001;
    ub = pi;

    [x_min, ~] = fminbnd(fun, lb, ub);
    phi = x_min;

    l(:) = [-r * cos(theta) * phi (0.5e0 * r * cos(theta) - sqrt(0.3e1) * r * sin(theta) / 0.2e1) * phi (0.5e0 * r * cos(theta) + sqrt(0.3e1) * r * sin(theta) / 0.2e1) * phi];
    l = l(:);

    z = (L/phi)*sin(phi);

end