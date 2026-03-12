clear
% continuum arm attributes
L = 0.317475;   % length of the module
r  = 0.013;   % radial offset of the 

% theta phi signal attributes
T = 1;
dt = 0.01;
npoints = floor(T/dt);
t = linspace(0,T,npoints);

phi = abs((pi/20) * sin(2*pi*t*(1/T)));
phi = phi';
theta = zeros(npoints,1);
theta(floor(npoints/2)+1:end,1) = pi;

l = config2length(theta, phi, r);


% Drawing cylinder
animatingSingleArm(l(2:3,:), L, r, 30);

figure(2)
plot(t,phi, t,theta);
grid on
axis tight
xlabel time
ylabel angle
legend phi theta

figure(3)
plot(t, l(1,:), '--k', t, l(2,:), '*b', t,l(3,:), 'o');
grid on
axis tight
xlabel time
ylabel length
legend l1 l2 l3

