% This script runs the simulation
% J = Jacobian_leg1(l, xi, B, L, r)
%   J   : Jacobian w.r.t. all the joint varirbles, and gloable frame
%         variables [3x14]
% 
% T = global_leg1HTM(l, xi, B, L, r)
%   T   : HTM at the 'xi' [4x4]
% 
% The number constraint legs = 4 and the constraint of the poisiton of the
% tip.
% Therefore, JC is [3x3 x 14]


close
clear
% load("C:\Users\dperera\OneDrive - Texas A&M University\Lab\Research\Quadruped\dynamic modeling\Matlab\gait_trajectory.mat")
sf = 0.8;
df = 0.7;


% robots parameters and the motion parameters
L = 0.317475;   % length of the module
r  = 0.013;   % radial offset of the 
rBody = 0.012;   % size of the body module
cycles = 10;  % number gait cycles
k = 3*4; % constraint legs
T = 0.5; % flexing time
dt = 0.01; % descretization
npoints = round(T/dt);
tvec = 0:dt:2*T;

% standing leg trajectory
std_task = config2task(0.0, pi/2, L);
std_len = config2length(0.0, pi/2, r);

% flexing body trajectory
[p, l] = bodyFlexingTrajectory(pi/20, T, dt, L, r);
% p = zeros()

b = l(2:3,1); % body bending
straight_pose = 1e-8*ones(2,1); % length changes for straigt pose


H = std_task(1,1); % the leg's X is worldframe Z. So the standing position.
B = zeros(6,npoints*cycles); % body frame trajectory
B(:,1) = [0;0;H; 0;0;0];  % [x y z roll pitch yaw]
COG = zeros(3,npoints*cycles);

qr = zeros(10, npoints*cycles); % [10x1]
qr(:,1) = [std_len(2); std_len(3); std_len(2); std_len(3); std_len(2); std_len(3); std_len(2); std_len(3); b(1); b(2)]; % [10x1]
wf_x = zeros(3, npoints*cycles);

% configuration parameters
leg1_config = zeros(2,size(qr,2));
leg2_config = zeros(2,size(qr,2));
leg3_config = zeros(2,size(qr,2));
leg4_config = zeros(2,size(qr,2));
body_config = zeros(2,size(qr,2));


count = 1;
for cycle=1:cycles % how many cycles of gait

        for i=1:npoints-1 % going through trajectory points
            b = l(2:3,i);
            Tbody_base = global_bodyHTM(B(:,count), b, 0.0, L, r);
            bp = Tbody_base * [p(:,i:i+1); ones(1,2)];
            bp = bp(1:3,:);
            
            % Book keeping
            wf_x(:,count) = Tbody_base(1:3, 4);

            jc1 = Jacobian_leg1_flex(qr(1:2, count), 1, B(:, count), b, 0.0, L, r); % leg 1 Jacobian
            jc2 = Jacobian_leg2_flex(qr(3:4, count), 1, B(:, count), b, 1.0, L, r); % leg 2 Jacobian
            jc3 = Jacobian_leg3_flex(qr(5:6, count), 1, B(:, count), b, 0.0, L, r); % leg 3 Jacobian
            jc4 = Jacobian_leg4_flex(qr(7:8, count), 1, B(:, count), b, 1.0, L, r); % leg 4 Jacobian
            jcb = Jacobian_body_flex(qr(9:10, count), 1, B(:, count), b, 1.0, L, r); % flexible body Jacobian

            % unconstraint leg
            % JC = [jc1; jc2; jc3; jc4]; % [12x16]
            % JJ = [JC; jcb]; % [15x16]
            % d_bp = bp(:,2) - bp(:,1);
            % W = diag([1000*ones(k, 1);100*ones(3,1)]);
            % Damped minimal norm solution for Null space (Tikhonov regularization)
            lambda = 5e-2;
            alpha = 1e-1;

            JC = [jc1; jc2; jc3; jc4];      % 12x16
            JC_hash = JC' / (JC*JC' + lambda^2*eye(size(JC,1)));
            N = eye(16) - JC_hash*JC;

            v_des = (bp(:,2) - bp(:,1))/dt;  % 3x1
            A = jcb * N;                                % 3x16

            % Penalize base motion strongly (indices 11:16 correspond to B)
            Wb = zeros(16,16);
            Wb(11:13,11:13) = diag([1e1, 1e1, 1e1]);              % penalize base translation
            Wb(14:16,14:16) = 1e1*eye(3);              % penalize base rotation
            
            Bmat = Wb * N;
            
            % Damped normal equation for z
            H = (A'*A) + alpha*(Bmat'*Bmat) + (1e-9)*eye(16);
            z = H \ (A' * v_des);
            qdot = N * z;
            
            resC = norm(JC*qdot)
            v_ach = jcb*qdot
            errV = norm(v_des - v_ach)
            q_next = [qr(:,count); B(:,count)] + qdot*dt;
            qr(:, count+1) = q_next(1:10);
            % B(:,count+1)  = q_next(11:16); 
            % qr(:, count+1) = qr(:, count) + [eye(10), zeros(10,6)] * ( pinv(W*JJ) * (W*[zeros(k,1); d_bp]) ); % [10x1]
            B(:, count+1) = B(:, count) - ( JC_hash(11:end,:) * ( JC(:, 1:10) * (qr(1:10, count+1) - qr(1:10, count)) ) );


            
            % compute the configuration variables for the qr
            leg1_config(:, count) = length2config(qr(1:2, count), r);
            leg2_config(:, count) = length2config(qr(3:4, count), r);
            leg3_config(:, count) = length2config(qr(5:6, count), r);
            leg4_config(:, count) = length2config(qr(7:8, count), r);
            body_config(:, count) = length2config(qr(9:10, count), r);

            count = count + 1;
            % if count >= 90
            %     disp(count);
            % end
        end
end

vec = 1:1:count;

figure(2)
plot(vec,B(1,1:count), vec,B(2,1:count), vec,B(3,1:count));
grid on
axis tight
xlabel 'iterations'
ylabel 'Cartesian'
legend 'X' 'Y' 'Z'

figure(3)
plot(vec,wf_x(1,1:count), vec,wf_x(2,1:count), vec,wf_x(3,1:count));
grid on
axis tight
xlabel 'iterations'
ylabel 'Cartesian'
legend 'X' 'Y' 'Z'


t = linspace(0,20,count-2)';
leg1_config = leg1_config(:, 1:count-2);
% leg1_config(:, count-2) = 0.5 * (leg1_config(:, 1) + leg1_config(:, count-2));

leg2_config = leg2_config(:, 1:count-2);
% leg2_config(:, count-2) = 0.5 * (leg2_config(:, 1) + leg2_config(:, count-2));

leg3_config = leg3_config(:, 1:count-2);
% leg3_config(:, count-2) = 0.5 * (leg3_config(:, 1) + leg3_config(:, count-2));

leg4_config = leg4_config(:, 1:count-2);
% leg4_config(:, count-2) = 0.5 * (leg4_config(:, 1) + leg4_config(:, count-2));

body_config = abs(body_config(:, 1:count-2));
% body_config(:, count-2) = 0.5 * (body_config(:, 1) + body_config(:, count-2));

[row, column] = size(qr);
if row>8
    b = qr(9:10,:);
    qr = qr(1:8, :);
end
animateQuadrupedFast(qr(:,1:end-2), B(:,1:end-2), b(:,1:end-2), L, r, rBody);
% animateQuadrupedFastToVideo(qr, B, b, L, r, rBody, "walking_v5");