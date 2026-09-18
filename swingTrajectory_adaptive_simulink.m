function gait = swingTrajectory_adaptive_simulink(stepLen, pos, r, Len, T, t)
%#codegen
% The trajectory consist of two parts: linear trajectory which attracts the
% legs toward the circular trajectory. If leg is outside the circle, it is
% drived towards the circle on the tangent line. 
% 
% Inputs:
%   stepLen : stride length (diameter of the semi-circle [constant] (m)
%   pos     : current position of the foot [2x1] (m)
%   r       : radial offset of the PMA from the backbone [constant] (m)
%   Len     : length of the PMA [constant] (m)
%   T       : swing time [constant] (s)
%   t       : globle time [constant] (s)
% 
% Output:
%   gait    : coordinates [x, y, z, 1] [4x1] (m)

% Fixed-size outputs
X   = zeros(1,2);
Y   = zeros(1,2);
Z   = zeros(1,2);
HTM = ones(1,2);
traj = zeros(12,1);

% Parameters
xb = 0.202;
yb = stepLen / 2;
x_b = [xb; yb];
R = stepLen / 2;
mode = 0; % where the foot is; outside/on/inside

horizontal_dist = abs(pos(2) - yb); % horizontal distance from foot to the circle center

% Compute for each of the two time instants
for i = 1:2
    ti = t(i);

    % tau in [0,1) with special handling at exact multiples of T:
    % original intent: if mod(t,T)==0 and t~=0, set tau=1 (end of cycle)
    m = mod(ti, T);
    tau = m / T;

    % Robust "is multiple of T" check (avoid == on doubles)
    if (abs(m) < 1e-12) && (ti ~= 0)
        tau = 1.0;
    end

    % Smooth time scaling s(tau)
    s = 3*tau*tau - 2*tau*tau*tau;

    if horizontal_dist > R
        alpha = acos(R/horizontal_dist); % angle to the tangent
        x_tangent = [xb-R*sin(alpha); yb-R*cos(alpha)]; % P2 position of the tangent
        
        L1 = norm(pos(1:2,1) - x_tangent); % length of the tangent
        L2 = R * (pi-alpha); % length of the remaining circle
        L = L1 + L2; % total distance
        sb = L1 / L; % ratio of tangent length for total lenght

        if s <= sb
            lambda = s / sb;
            Xi = pos(1:2,1) + lambda * (x_tangent - pos(1:2,1));
        else
            mu = (s - sb) / (1-sb);
            theta_mu = alpha + mu * (pi - alpha);
            Xi = x_b - R * [sin(theta_mu); cos(theta_mu)];
        end

    elseif abs(horizontal_dist - R) <= R * 0.01 
        % is foot is near the circle (close  to 1% of the radius)
        Xi = x_b - R * [sin(pi*s); cos(pi*s)]; % typical circle
    else
        % inside the circle
        p_end = [xb; yb + R];
        new_R = norm(pos(1:2,1) - p_end) / 2;
        new_x_b = (pos(1:2,1) + p_end) / 2;
        Xi = new_x_b - new_R * [sin(pi*s); cos(pi*s)];
    end
    
    if i == 1
        traj = [mode;
                sb;
                pos(1,1);
                pos(2,1);
                x_tangent(1);
                x_tangent(1);
                alpha;
                pi;
                x_b(1);
                x_b(2);
                R;
                t_0;
                ];
    end

    % % Swing trajectory in task frame
    % U  = R * sin(pi*s);     % lift profile
    % Xi = xb - U;               % encode lift into X as you did
    % Yi = yb - stepLen*s;       % forward progression

    X(i) = Xi(1);
    Y(i) = Xi(2);

    % Get Z from task2length (must be codegen-safe)
    [~, Zi] = task2length([X(i); Y(i)], r, Len);
    Z(i) = Zi;
end

% Return 4x2 (fixed size)
gait = [X; Y; Z; HTM];

end
