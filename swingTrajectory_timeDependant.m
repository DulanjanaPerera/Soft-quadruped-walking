function  gait = swingTrajectory_timeDependant(stepLen, lift, r, L, T, t)
% r = 0.013;
% L = 0.278;
k = (mod(t,T) == 0);
notZero = (t ~= 0);
tau = t./T - floor(t./T) + k .* notZero;
HTM = ones(size(t));

x0 = 0.19;
y0 = stepLen/2;
% y0 = 0.0;

% time-normalized parameter
% tau = 0:0.01:1;

% smooth time scaling: s(tau)
s = 3*tau.^2 - 2*tau.^3;

% Planar swing in your task frame (check which axis is "up" for task2length)
X = x0;                              % keep constant (example)
Y = y0-stepLen*s;                  % forward progression
U = lift * sin(pi*s);                % "lift" component in the plane

% If your task2length expects [X;Y], decide where U goes:
% Option A: treat X as vertical (lift), Y as forward:
X = x0 - U;

% Convert to internal variables and compute Z (as your function returns)
numPoints = numel(s);
Z = zeros(1,numPoints);

for i = 1:numPoints
    [~, Z(i)] = task2length([X(i); Y(i)], r, L);
end

gait = [X; Y; Z; HTM];

% figure; plot(tau, X, tau, Y, tau, Z); grid on; legend('X','Y', 'Z'); xlabel('\tau');
% 
% figure;
% plot3(X, Y, Z, '*k'); grid on; axis image;
% xlabel('x'); ylabel('y'); zlabel('z');
% title('Swing trajectory (mapped output)');

% save("gait_trajectory.mat", "tau", "s", "X", "Y", "Z");

end
