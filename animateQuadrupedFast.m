function animateQuadrupedFast(qr, B, L, rLeg, rBody)
% animateQuadrupedFast
% Fast animation by updating vertices (no re-creating surfaces).

N = size(qr,2);
if size(B,2) ~= N
    error('qr and B must have same number of columns (time steps).');
end

% Init plot objects once (fixed axes set here)
h = initContinuumRobotPlot(L, rLeg, rBody, ...
    'nXi', 20, 'nSides', 16, 'BodyAxis', 'x', 'BodySign', -1);

% Playback speed control
fps = 30;
dtPlot = 1/fps;
tLast = tic;

for k = 1:N
    q1 = qr(1:2,k);
    q2 = qr(3:4,k);
    q3 = qr(5:6,k);
    q4 = qr(7:8,k);

    updateContinuumRobotPlot(h, {q1,q2,q3,q4}, B(:,k));
    title(sprintf('Step %d / %d', k, N));

    % Throttle
    elapsed = toc(tLast);
    if elapsed < dtPlot
        pause(dtPlot - elapsed);
    end
    tLast = tic;
end
end
