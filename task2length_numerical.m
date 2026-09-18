function [l, z] = task2length(p, r, L)
%#codegen
    if nargin < 3
        L = 0.278;
        r = 0.013;
    end

    theta = atan2(p(2), p(1));
    rho_t = hypot(p(1), p(2)); % target rho

    % Max reachable rho on (0,pi] for this model
    rho_max = 2*L/pi;

    if rho_t < 1e-12
        phi = 0.0;
    elseif rho_t >= rho_max
        phi = pi;  % clamp to max bend (unreachable target)
    else
        a = 1e-6;
        b = pi;

        % Bisection with fixed iterations (deterministic)
        for it = 1:50
            m = 0.5*(a+b);

            rho_m = (L/m)*(1 - cos(m));
            g = rho_m - rho_t;

            if g > 0
                b = m;   % too much rho -> phi too big
            else
                a = m;   % too little rho -> phi too small
            end
        end
        phi = 0.5*(a+b);
    end

    % actuator lengths
    l = zeros(3,1);
    l(1) = -r*cos(theta)*phi;
    l(2) = (0.5*r*cos(theta) - (sqrt(3)/2)*r*sin(theta))*phi;
    l(3) = (0.5*r*cos(theta) + (sqrt(3)/2)*r*sin(theta))*phi;

    % z coordinate
    if phi < 1e-6
        z = L;
    else
        z = (L/phi)*sin(phi);
    end
end
