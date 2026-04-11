function traj = swingTrajectory_initializer_simulink(stepLen, pos, t)
%#codegen
% This function initialize the parameters for adaptive swing gait where
% the trajectory is updated according to the foot position. The function
% determined the swing type; outside leg swing, normal semi-circle arc, and
% inside small semi-circle swing. Then function returns the parameters for
% the trajectory.
% 
% Inputs:
%   stepLen : stride length (diameter of the semi-circle [constant] (m)
%   pos     : current position of the foot [2x1] (m)
%   t       : globle time [constant] (s)
% 
% Output:
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
%             ]

% Fixed-size outputs
traj = zeros(12,1);
x_tangent = zeros(2,1);

% Parameters
xb = 0.202;
yb = 0.0;
x_b = [xb; yb];
R = stepLen / 2;
mode = 0; % where the foot is; outside/on/inside
sb = 0; % length ratio betweet tangent and arc
alpha = 0;  % arc angle of the tangent

horizontal_dist = abs(pos(2) - yb); % horizontal distance from foot to the circle center

if horizontal_dist > R
    mode = 1;
    alpha = acos(R/horizontal_dist); % angle to the tangent
    x_tangent = [xb-R*sin(alpha); yb-R*cos(alpha)]; % P2 position of the tangent
    
    L1 = norm(pos(1:2,1) - x_tangent); % length of the tangent
    L2 = R * (pi-alpha); % length of the remaining circle
    L = L1 + L2; % total distance
    if L > 1e-12
        sb = L1 / L; % ratio of tangent length for total lenght
    else
        sb = 0.0;
    end 
elseif abs(horizontal_dist - R) <= R * 0.01 
    % is foot is near the circle (close  to 1% of the radius)
    mode = 2;
else
    % inside the circle
    mode = 3;
    p_end = [xb; yb + R];
    R = norm(pos(1:2,1) - p_end) / 2;
    x_b = (pos(1:2,1) + p_end) / 2;
end

traj = [mode;
        sb;
        pos(1,1);
        pos(2,1);
        x_tangent(1);
        x_tangent(2);
        alpha;
        pi;
        x_b(1);
        x_b(2);
        R;
        t;
        ];

end
