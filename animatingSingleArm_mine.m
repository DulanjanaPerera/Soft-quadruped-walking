function animatingSingleArm(l, L, r)
[~, columns] = size(l);
xii = linspace(0,1,20);
np = 21;
figure(1);
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
    surf(Xx, Yy, Zz,'FaceColor', 'flat', 'EdgeColor', 'k');
    colormap([0.3 0.6 0.3]);  % green base
    clim([0 1])
    hold on
    fill3(Xx(1,:), Yy(1,:), Zz(1,:), 'g');
    fill3(Xx(end,:), Yy(end,:), Zz(end,:), 'g');
    hold off
    xlabel x
    ylabel y
    zlabel z
    grid on
    axis equal
    xlim([-0.2 0.2])
    ylim([-0.2 0.2])
    zlim([0 0.5])

    camlight; lighting gouraud; rotate3d on;
    drawnow limitrate;
    pause(0.01);
end
end