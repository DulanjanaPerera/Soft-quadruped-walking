function [l,z] = task2length_old(p, r, L)
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
    % theta_l = atan2(lp(2)-lp(1), sqrt(3)*(lp(2)+lp(1)));
    % phi_l = wrapToPi((2/r)*sqrt((lp(1)^2 + lp(2)^2 + lp(1)*lp(2)) / 3));

    % fun = @(x)sqrt(((-cos(theta) * cos(x) * L / x + cos(theta) * L / x)-p(1))^2 +...
    %     ((-sin(theta) * cos(x) * L / x + sin(theta) * L / x)-p(2))^2 +...
    %     ((sin(x) * L / x)-p(3))^2);
    
    % fun = @(x)1*sqrt(( (-cos(theta) * cos(x) * L / x + cos(theta) * L / x) - p(1) )^2 ...
    %                  + ( (-sin(theta) * cos(x) * L / x + sin(theta) * L / x) - p(2) )^2 ...
    %                  + ( cos(x) - 1 + ( (x* sqrt(p(1)^2 + p(2)^2)) / L ) )^2);

    fun = @(x)1*sqrt(  ( (-cos(theta) * cos(x) * L / x + cos(theta) * L / x) - p(1) )^2 ...
                     + ( (-sin(theta) * cos(x) * L / x + sin(theta) * L / x) - p(2) )^2 ...
                     + ( (1-cos(x))/x - (sqrt(p(1)^2+p(2)^2)/L) )^2);

    % phi bounds are (0, pi]
    lb = 0.0001;
    ub = pi;
    
    % F = @(x) sqrt(p(1).^2+p(2).^2)-sqrt(L.^2*(1-cos(x)).^2./(x.^2));
    % x=fsolve(F,[0.000001;pi]);
    % phi=x(1);

    options = optimset('PlotFcns',@optimplotfval);
    [x_min, ~] = fminbnd(fun, lb, ub);
    phi = x_min;

    l(:) = [-r * cos(theta) * phi (0.5e0 * r * cos(theta) - sqrt(0.3e1) * r * sin(theta) / 0.2e1) * phi (0.5e0 * r * cos(theta) + sqrt(0.3e1) * r * sin(theta) / 0.2e1) * phi];
    l = l(:);

    z = (L/phi)*sin(phi);

end