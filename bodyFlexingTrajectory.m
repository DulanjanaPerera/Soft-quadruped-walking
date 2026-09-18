function [p, l] = bodyFlexingTrajectory(phi_bend, T, dt, L, r)
% This function computes the trajectories of taks space variable and joint space variable
% for body flexing.
%
% Input:
%   T

% theta phi signal attributes
npoints = floor(T/dt);
t = linspace(0,T,npoints);

phi = abs(phi_bend * sin(2*pi*t*(1/T)));
phi = phi';
theta = zeros(npoints,1);
theta(floor(npoints/2)+1:end,1) = pi;

p = config2task(theta, phi, L);
l = config2length(theta, phi, r);

end