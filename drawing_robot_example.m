% Example parameters
close
clear
load("C:\Users\dperera\OneDrive - Texas A&M University\Lab\Research\Quadruped\dynamic modeling\Matlab\gait_trajectory.mat")
L = 0.278;        % m
rLeg  = 0.013;   % m
rBody = 0.012;   % m
H = X(1);
B = [0;0;H; 0;0;0];  % [x y z roll pitch yaw] or your convention

[l,~] = task2length([H; 0], rLeg, L);
[l2,~] = task2length([X(1); Y(1)], rLeg, L);

q1 = [l(2); l(3)];
q4 = [l2(2); l2(3)];
q3 = [l(2); l(3)];
q2 = [l(2); l(3)];


drawContinuumRobotInstance({q1,q2,q3,q4}, B, L, rLeg, rBody, ...
    'nXi', 50, 'nSides', 32, 'BodyAxis', 'x', 'BodySign', -1, 'Hold', false);
