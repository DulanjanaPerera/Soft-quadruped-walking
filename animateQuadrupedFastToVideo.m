function animateQuadrupedFastToVideo(qr, B, b, L, rLeg, rBody, dt, outFile)

N = size(qr,2);
if size(B,2) ~= N
    error('qr and B must have same number of columns (time steps).');
end

fps = 30;
v = VideoWriter(outFile, 'MPEG-4');
v.FrameRate = fps;
open(v);

h = initContinuumRobotPlot(L, rLeg, rBody, ...
    'nXi', 50, 'nSides', 32, 'BodyAxis', 'x', 'BodySign', -1);

% Contact detection + CoG settings
h.contactZThresh = 0.01;     % meters (tune: 1e-3 .. 1e-2)
h.useRelativeGround = true;   % more robust than absolute z=0
h.groundPad = 0.002;          % extra margin above estimated ground

fps = 30;
dtPlot = 1/fps;
tLast = tic;

time_vid = 0;

for k = 1:N
    q1 = qr(1:2,k);
    q2 = qr(3:4,k);
    q3 = qr(5:6,k);
    q4 = qr(7:8,k);

    if size(b,2) == N
        bk = b(:,k);
    else
        bk = b;
    end

    updateContinuumRobotPlot(h, {q1,q2,q3,q4}, B(:,k), bk, qr(:,k));
    % title(sprintf('Step %d / %d', k, N));
    title(sprintf('Time %0.2f s', time_vid));
    time_vid = time_vid + dt;    

    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);
end
