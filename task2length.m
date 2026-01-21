function [l, z] = task2length(p, r, L)
    if nargin < 2
        L = 0.278;
        r = 0.013;
    end
    
    theta = atan2(p(2), p(1));
    rho = sqrt(p(1)^2 + p(2)^2); % radial distance
    
    % Better initial guess based on small-angle approximation
    if rho < 1e-6
        phi_init = 1e-3;
    else
        phi_init = min(pi, max(1e-3, 2*rho/L)); % better initial guess
    end
    
    % Objective function (minimize distance to target)
    fun = @(phi) costFunction(phi, theta, L, p);
    
    % Constraints
    lb = 1e-4;  % avoid singularity at phi=0
    ub = pi;
    
    % Nonlinear constraint: ensure reachable workspace
    nonlcon = @(phi) workspaceConstraint(phi, L, rho);
    
    % fmincon options
    options = optimoptions('fmincon', ...
        'Display', 'off', ...
        'Algorithm', 'interior-point', ...
        'MaxIterations', 200, ...
        'TolFun', 1e-8, ...
        'TolX', 1e-8);
    
    % Solve
    [phi, ~, exitflag] = fmincon(fun, phi_init, [], [], [], [], lb, ub, nonlcon, options);
    
    % Check if solution is valid
    if exitflag < 1
        warning('Optimization did not converge. Point may be unreachable.');
    end
    
    % Compute actuator lengths
    l = zeros(3, 1);
    l(1) = -r * cos(theta) * phi;
    l(2) = (0.5 * r * cos(theta) - sqrt(3) * r * sin(theta) / 2) * phi;
    l(3) = (0.5 * r * cos(theta) + sqrt(3) * r * sin(theta) / 2) * phi;
    
    % Compute z coordinate (handle small phi)
    if phi < 1e-6
        z = L; % straight configuration
    else
        z = (L/phi) * sin(phi);
    end
end

function cost = costFunction(phi, theta, L, p)
    % Forward kinematics
    if phi < 1e-6
        x_fk = 0;
        y_fk = 0;
    else
        x_fk = (L/phi) * (1 - cos(phi)) * cos(theta);
        y_fk = (L/phi) * (1 - cos(phi)) * sin(theta);
    end
    
    % Squared error
    cost = (x_fk - p(1))^2 + (y_fk - p(2))^2;
end

function [c, ceq] = workspaceConstraint(phi, L, rho)
    % Inequality constraints (c <= 0)
    c = [];
    
    % Equality constraint: radial distance must match
    if phi < 1e-6
        rho_fk = 0;
    else
        rho_fk = (L/phi) * (1 - cos(phi));
    end
    
    ceq = rho_fk - rho; % must be zero
end