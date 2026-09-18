function animateQuadrupedToVideo(qr, B, L, rLeg, rBody, outFile)

N = size(qr,2);
if size(B,2) ~= N
    error('qr and B must have same number of columns (time steps).');
end

fps = 30;
v = VideoWriter(outFile, 'MPEG-4');
v.FrameRate = fps;
open(v);

figure(1); clf;
axis equal; grid on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');

% Fix bounds to avoid jitter in the video
xlim([-0.2 0.5]); ylim([-0.3 0.3]); zlim([0 0.4]);

for k = 1:N

    q1 = qr(1:2,k);
    q2 = qr(3:4,k);
    q3 = qr(5:6,k);
    q4 = qr(7:8,k);

    drawContinuumRobotInstance({q1,q2,q3,q4}, B(:,k), L, rLeg, rBody, ...
        'nXi', 50, 'nSides', 32, 'BodyAxis', 'x', 'BodySign', -1, 'Hold', false);

    title(sprintf('Step %d / %d', k, N));
    drawnow;

    frame = getframe(gcf);
    writeVideo(v, frame);

end

close(v);
end
