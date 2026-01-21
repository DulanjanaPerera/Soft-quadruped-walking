function animateQuadrupedFastToVideo(qr, B, L, rLeg, rBody, outFile)

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

for k = 1:N
    q1 = qr(1:2,k);
    q2 = qr(3:4,k);
    q3 = qr(5:6,k);
    q4 = qr(7:8,k);

    updateContinuumRobotPlot(h, {q1,q2,q3,q4}, B(:,k));
    title(sprintf('Step %d / %d', k, N));

    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);
end
