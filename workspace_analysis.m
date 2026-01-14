lmin = -0.028;
lmax = 0.028;

numPoints = 100; % Define the number of points for the range
linSpace = linspace(lmin, lmax, numPoints); % Create a linearly spaced vector

p = zeros(numPoints,numPoints,3);

for l1=1:numPoints
    for l2=1:numPoints
        temp_htm = singleHTM([linSpace(l1);linSpace(l2)], 1, 0.278, 0.013);
        p(l1,l2,1) = temp_htm(1,4);
        p(l1,l2,2) = temp_htm(2,4);
        p(l1,l2,3) = temp_htm(3,4);
    end
end

figure(1)
plot3(p(:,:,1), p(:,:,2), p(:,:,3), '*k')
grid on
axis equal
xlabel x
ylabel y;
zlabel z;
title('3D Plot of HTM Outputs');