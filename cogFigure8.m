function COG_d = cogFigure8(k, npoints, cycle, swingLeg, footPos, stepLen)
% footPos: 3x4 matrix, columns are [leg1 leg2 leg3 leg4] foot positions in world frame.
% Output COG_d: 3x1 desired CoG displacement increment target in world frame.

% Top-view foot positions
p1 = footPos(:,1);
p2 = footPos(:,2);
p3 = footPos(:,3);
p4 = footPos(:,4);

% Approximate polygon center
xc = 0.25 * (p1 + p2 + p3 + p4);

% Define forward and lateral directions in world frame.
% Based on your code, B(1) appears forward and B(2) lateral.
e_fb = [1; 0; 0];   % forward
e_lr = [0; 1; 0];   % left-right

% Phase over one full four-leg gait cycle
% phase = 2*pi * mod(k-1, 4*npoints) / (4*npoints);
phase = 2*pi * mod(k-1, 4*npoints) / (4*npoints);

% Amplitudes
alpha_lr = 0.3; % left-to-right swing distance
alpha_fb = 0.3; % front-to-back swing distance

A_lr = alpha_lr * stepLen;
A_fb = alpha_fb * stepLen;

% if swingLeg == 1 || swingLeg == 2
%     sideSign = -1;   % move CoG away from left/front-side swing, adjust if convention differs
% else
%     sideSign =  1;
% end

% COG_d = xc ...
%       + sideSign * A_lr * e_lr * sin(phase) ...
%       + A_fb * e_fb * sin(2*phase);

% Figure-8 CoG trajectory
% COG_d = xc ...
%       + A_lr * e_lr * sin(phase + pi/2) ...
%       + A_fb * e_fb * sin(2*phase);
COG_d = xc ...
      + A_lr * e_lr * sin(phase -pi/2) ...
      + A_fb * e_fb * sin(2*phase - pi);

% Keep vertical CoG unchanged in this task
% COG_d(3) = xc(3);
COG_d(3) = 0.0;
end