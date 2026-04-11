function gait = swingTrajectory_evaluator_simulink(r, Len, T, t, traj)
%#codegen
% The trajectory consist of two parts: linear trajectory which attracts the
% legs toward the circular trajectory. If leg is outside the circle, it is
% drived towards the circle on the tangent line. 
% 
% The leg frame is:
% 
%   o ------ > Y
%   |
%   |
%   v
%   X
% 
% Inputs:
%   r       : radial offset of the PMA from the backbone [constant] (m)
%   Len     : length of the PMA [constant] (m)
%   T       : swing time [constant] (s)
%   t       : globle time [constant] (s)
%   traj    : [ mode: where the leg is outside/on/inside of the circle;
%               sb: length scale of the tangent and ;
%               pos(1,1): Initial leg position x;
%               pos(2,1): Initial leg position y;
%               x_tangent(1): tangent position x;
%               x_tangent(1): tangent position y;
%               alpha: tangent arc angle;
%               pi: half circle arc angle;
%               x_b(1): circle x coordinate;
%               x_b(2): circle y coordinate;
%               R: circle radius;
%               t: global time of the leg initiation of the swing;
%               side: what side (1-left, -1-right)
%             ]
% 
% Output:
%   gait    : leg frame coordinates [4,2] (m)

% Fixed-size outputs
X   = zeros(1,2);
Y   = zeros(1,2);
Z   = zeros(1,2);
HTM = ones(1,2);
x_tangent = zeros(2,1);
pos = zeros(2,1);
x_b = zeros(2,1);

% horizontal_dist = abs(pos(2) - yb); % horizontal distance from foot to the circle center

mode = traj(1);
sb = traj(2);
pos(1,1) = traj(3);
pos(2,1) = traj(4);
x_tangent(1) = traj(5);
x_tangent(2) = traj(6);
alpha = traj(7);
theta_f = traj(8);
x_b(1) = traj(9);
x_b(2) = traj(10);
R = traj(11);
t_init = traj(12);
side = traj(13); % side of the robot
  

% Compute for each of the two time instants
for i = 1:2
    ti = t(i);

    % tau in [0,1) with special handling at exact multiples of T:
    % original intent: if mod(t,T)==0 and t~=0, set tau=1 (end of cycle)
    tau = (ti - t_init) / T;
    
    if tau < 0.0
        tau = 0.0;
    elseif tau > 1.0
        tau = 1.0;
    end

    % Smooth time scaling s(tau)
    s = 3*tau*tau - 2*tau*tau*tau;

    if mode == 1
        % determined from the initial sdfsds initializer fucntion
        % alpha = acos(R/horizontal_dist); % angle to the tangent
        % x_tangent = [xb-R*sin(alpha); yb-R*cos(alpha)]; % P2 position of the tangent
        % 
        % L1 = norm(pos(1:2,1) - x_tangent); % length of the tangent
        % L2 = R * (pi-alpha); % length of the remaining circle
        % L = L1 + L2; % total distance
        % sb = L1 / L; % ratio of tangent length for total lenght

        if (sb > 1e-12) && s <= sb 
            lambda = s / sb;
            Xi = pos(1:2,1) + lambda * (x_tangent - pos(1:2,1));
        else
            if (1.0 - sb) > 1e-12
                mu = (s - sb) / (1 - sb);
            else
                mu = 1.0;
            end
            theta_mu = alpha + mu * (theta_f - alpha);
            Xi = x_b + [-R * sin(theta_mu);
                         side * R * cos(theta_mu)];
        end

    elseif mode == 2 
        % is foot is near the circle (close  to 1% of the radius)
        Xi = x_b + [-R * sin(pi*s);
                     side * R * cos(pi*s)]; % typical circle
    else
        % inside the circle
        % p_end = [xb; yb + R]; % determined from the initial sdfsds initializer fucntion
        % new_R = norm(pos(1:2,1) - p_end) / 2; determined from the initial sdfsds initializer fucntion
        % new_x_b = (pos(1:2,1) + p_end) / 2; determined from the initial sdfsds initializer fucntion
        Xi = x_b + [-R * sin(pi*s);
                     side * R * cos(pi*s)];
    end

    X(i) = Xi(1);
    Y(i) = Xi(2);

    % Get Z from task2length (must be codegen-safe)
    [~, Zi] = task2length([X(i); Y(i)], r, Len);
    Z(i) = Zi;
end

% Return 4x2 (fixed size)
gait = [X; Y; Z; HTM];

end
