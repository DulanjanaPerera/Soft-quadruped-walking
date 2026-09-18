function animatingSingleArm(l, L, r, fps)
[~, columns] = size(l);
xii = linspace(0,1,20);
np = 21;
% Setup figure
figure(1); clf;
ax = gca;


% Enable rotation BEFORE the loop
rotate3d(ax, 'on');


for j=1:columns
    [X, Y, Z] = cylinder(0.02);
    Xx = zeros(length(xii),np);
    Yy = zeros(length(xii),np);
    Zz = zeros(length(xii),np);

    for i=1:length(xii)
            xi = xii(i);
            T = singleHTM(l(:,j), xi, L, r);
            p = [X(1,:); Y(1,:); Z(1,:); ones(size(X(1,:)))];
            P = T*p;
            Xx(i,:) = P(1,:);
            Yy(i,:) = P(2,:);
            Zz(i,:) = P(3,:);
    end

    surf(ax, Xx, Yy, Zz,'FaceColor', [0.3 0.6 0.3], 'EdgeColor', 'k');
    hold(ax,'on');
    fill3(ax, Xx(1,:), Yy(1,:), Zz(1,:), 'g');
    fill3(ax, Xx(end,:), Yy(end,:), Zz(end,:), 'g');
    hold(ax, 'off');


    xlabel('x'); ylabel('y'); zlabel('z');
    grid(ax, 'on'); axis(ax, 'equal');
    xlim(ax, [-0.2 0.2]); ylim(ax, [-0.2 0.2]); zlim(ax, [0 0.5]);
    camlight; lighting gouraud;
    drawnow limitrate;  % Update graphics, limit to monitor refresh rate
        
    % Pause to control frame rate
    pause(1/fps);
end
end