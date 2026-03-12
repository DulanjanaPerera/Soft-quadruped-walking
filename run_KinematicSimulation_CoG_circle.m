% This script runs the simulation
% J = Jacobian_leg1(l, xi, B, L, r)
%   J   : Jacobian w.r.t. all the joint varirbles, and gloable frame
%         variables [3x14]
% 
% T = global_leg1HTM(l, xi, B, L, r)
%   T   : HTM at the 'xi' [4x4]
% 
% The number constraint legs = 3 and the constraint of the poisiton of the
% tip.
% Therefore, JC is [3x3 x 14]


close
clear
% load("C:\Users\dperera\OneDrive - Texas A&M University\Lab\Research\Quadruped\dynamic modeling\Matlab\gait_trajectory.mat")

% robots parameters and the motion parameters
L = 0.278;   % length of the module
r  = 0.013;   % radial offset of the 
rBody = 0.012;   % size of the body module
cycles = 2;  % number gait cycles
k = 9; % constraint legs

stepLen_l = 0.04;     % step length
lift    = 0.01;
[X_l, Y_l, Z_l] = swingTrajectory(stepLen_l, lift); % left side trajectory
stance_wf_l = [0.0008;0;0]; % world frame stance

stepLen_r = 0.02;     % step length
lift    = 0.01;
[X_r, Y_r, Z_r] = swingTrajectory(stepLen_r, lift); % left side trajectory
stance_wf_r = [0.0008;0;0]; % world frame stance

dp_cog = [0.0001;-0.00002;0.00000];

stance = [0;0;-0.0008; 1]; % leg frame stance
b = -2e-3*ones(2,1); % body bending
straight_pose = 1e-8*ones(2,1); % length changes for straigt pose

npoints = length(X_l);
P_d_l = [X_l; Y_l; Z_l; ones(1,npoints)]; % left side trajectory
P_d_r = [X_r; Y_r; Z_r; ones(1,npoints)]; % right side trajectory


H = X_l(1); % the leg's X is worldframe Z. So the standing position.
B = zeros(6,npoints*4*cycles); % body frame trajectory
B(:,1) = [0;0;H; 0;0;0];  % [x y z roll pitch yaw] or your convention
COG = zeros(3,npoints*4*cycles);


[l,~] = task2length([H; 0], r, L);
[l2,~] = task2length([X_l(1); Y_l(1)], r, L);


qr = zeros(8, npoints*4*cycles);
qr(:,1) = [l(2); l(3); l2(2); l2(3); l(2); l(3); l(2); l(3)];
wf_x = zeros(3, npoints*4*cycles);

% configuration parameters
leg1_config = zeros(2,size(qr,2));
leg2_config = zeros(2,size(qr,2));
leg3_config = zeros(2,size(qr,2));
leg4_config = zeros(2,size(qr,2));


count = 1;
for cycle=1:cycles % how many cycles of gait
    
    for legs=[2,1,4,3] % leg sequence

        for i=1:npoints - 1 % going through trajectory points

            if legs==1
                Tbase = global_leg1HTM(straight_pose, 0.0, B(:, count), b, 0.0, L, r);
                P_d_w = Tbase * P_d_l;
                stance_w = Tbase * stance;

                stance_w = stance_w(1:3,1);
                P_d_w = P_d_w(1:3,:);
            elseif legs ==2
                Tbase = global_leg2HTM(straight_pose, 0.0, B(:, count), b, 1.0, L, r);
                P_d_w = Tbase * P_d_l;
                stance_w = Tbase * stance;

                stance_w = stance_w(1:3,1);
                P_d_w = P_d_w(1:3,:);
            elseif legs==3
                flipy = eye(4);
                flipy(2,2) = -1;
                Tbase = global_leg3HTM(straight_pose, 0.0, B(:, count), b, 0.0, L, r);
                P_d_w = Tbase * flipy * P_d_r;
                stance_w = Tbase * flipy * stance;

                stance_w = stance_w(1:3,1);
                P_d_w = P_d_w(1:3,:);
            elseif legs==4
                flipy = eye(4);
                flipy(2,2) = -1;
                Tbase = global_leg4HTM(straight_pose, 0.0, B(:, count), b, 1.0, L, r);
                P_d_w = Tbase * flipy * P_d_r;
                stance_w = Tbase * flipy * stance;

                stance_w = stance_w(1:3,1);
                P_d_w = P_d_w(1:3,:);
            end
            % compute the configuration variables for the qr
            leg1_config(:, count) = length2config(qr(1:2, count), r);
            leg2_config(:, count) = length2config(qr(3:4, count), r);
            leg3_config(:, count) = length2config(qr(5:6, count), r);
            leg4_config(:, count) = length2config(qr(7:8, count), r);

            wf_x(:,count) = P_d_w(:,i);

            jc1 = Jacobian_leg1(qr(1:2, count), 1, B(:, count), b, 0.0, L, r);
            jc2 = Jacobian_leg2(qr(3:4, count), 1, B(:, count), b, 1.0, L, r);
            jc3 = Jacobian_leg3(qr(5:6, count), 1, B(:, count), b, 0.0, L, r);
            jc4 = Jacobian_leg4(qr(7:8, count), 1, B(:, count), b, 1.0, L, r);
            
            % unconstraint leg
            if legs==1
                JC = [jc2; jc3; jc4];
                J = jc1;
            elseif legs ==2
                JC = [jc1; jc3; jc4];
                J = jc2;
            elseif legs==3
                JC = [jc1; jc2; jc4];
                J = jc3;
            elseif legs==4
                JC = [jc1; jc2; jc3];
                J = jc4;
            end
            Jcog = Jacobian_cog(qr(1:8), 1, B(:, count), b, 1.0, L, r);

            dp = P_d_w(:,i+1) - P_d_w(:,i);

            JJ = [JC;J;Jcog];
            W = diag([1000*ones(k, 1);100*ones(3,1); 10*ones(3,1)]);

            qr(:, count+1) = qr(:, count) + [eye(8), zeros(8,6)] * ( (W*JJ) \ (W * [zeros(k,1); dp; dp_cog]) );
            B(:, count+1) = B(:, count) - ( JC(:,9:end) \ ( JC(:, 1:8) * (qr(:, count+1) - qr(:, count)) ) );


            count = count + 1;
        end
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


t = linspace(0,10,count-1)';
leg1_config = leg1_config(:, 1:count-1);
leg1_config(:, count-1) = 0.5 * (leg1_config(:, 1) + leg1_config(:, count-1));

leg2_config = leg2_config(:, 1:count-1);
leg2_config(:, count-1) = 0.5 * (leg2_config(:, 1) + leg2_config(:, count-1));

leg3_config = leg3_config(:, 1:count-1);
leg3_config(:, count-1) = 0.5 * (leg3_config(:, 1) + leg3_config(:, count-1));

leg4_config = leg4_config(:, 1:count-1);
leg4_config(:, count-1) = 0.5 * (leg4_config(:, 1) + leg4_config(:, count-1));

body_config = length2config(b, r);

% leg1_theta = [leg1_config(1,:)', t];
% leg1_phi = [leg1_config(2,:)', t];
% 
% leg2_theta = [leg2_config(1,:)', t];
% leg2_phi = [leg2_config(2,:)', t];
% 
% leg3_theta = [leg3_config(1,:)', t];
% leg3_phi = [leg3_config(2,:)', t];
% 
% leg4_theta = [leg4_config(1,:)', t];
% leg4_phi = [leg4_config(2,:)', t];

% leg1_theta = timeseries(leg1_config(1,:), t);
% leg1_phi = timeseries(leg1_config(2,:), t);
% 
% leg2_theta = timeseries(leg2_config(1,:), t);
% leg2_phi = timeseries(leg2_config(2,:), t);
% 
% leg3_theta = timeseries(leg3_config(1,:), t);
% leg3_phi = timeseries(leg3_config(2,:), t);
% 
% leg4_theta = timeseries(leg4_config(1,:), t);
% leg4_phi = timeseries(leg4_config(2,:), t);

animateQuadrupedFast(qr, B, b, L, r, rBody)
% animateQuadrupedFastToVideo(qr, B, b, L, r, rBody, "walking_v5");