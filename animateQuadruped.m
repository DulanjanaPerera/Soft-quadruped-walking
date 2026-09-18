function animateQuadruped(qr, B, L, rLeg, rBody)

N = size(qr,2);
if size(B,2) ~= N
    error('qr and B must have same number of columns (time steps).');
end

figure(1); clf;

% Optional: set a consistent view and axis limits once
ax = gca;
axis equal; grid on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');

% If you know workspace bounds, set them to prevent auto-rescale flicker
xlim([-0.3 0.5]); ylim([-0.3 0.3]); zlim([0 0.4]);

% Animation timing
fps = 30;
dtPlot = 1/fps;
tLast = tic;

for k = 1:N

    % Unpack 2-DoF leg parameters
    q1 = qr(1:2,k);
    q2 = qr(3:4,k);
    q3 = qr(5:6,k);
    q4 = qr(7:8,k);

    % Redraw the robot for this time step
    drawContinuumRobotInstance({q1,q2,q3,q4}, B(:,k), L, rLeg, rBody, ...
        'nXi', 50, 'nSides', 32, 'BodyAxis', 'x', 'BodySign', -1, 'Hold', false);

    title(sprintf('Step %d / %d', k, N));

    drawnow;

    % Throttle to ~fps (prevents running too fast)
    elapsed = toc(tLast);
    if elapsed < dtPlot
        pause(dtPlot - elapsed);
    end
    tLast = tic;

end
end
