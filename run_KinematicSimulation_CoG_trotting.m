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
cycles = 5;  % number gait cycles
k = 6; % constraint legs joints
swing_legs = 12 - k; % swing legs joints

stepLen_l = 0.001;     % step length
lift    = 0.01;
[X_l, Y_l, Z_l] = swingTrajectory(stepLen_l, lift); % left side trajectory
stance_wf_l = [0.0008;0;0]; % world frame stance

stepLen_r = 0.001;     % step length
lift    = 0.01;
[X_r, Y_r, Z_r] = swingTrajectory(stepLen_r, lift); % left side trajectory
stance_wf_r = [0.0008;0;0]; % world frame stance

dp_cog = [0.00001;-0.000;0.00000];

stance = [0;0;-0.0008; 1]; % leg frame stance
b = 5e-8*ones(2,1); % body bending
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
wf_x = zeros(3, npoints*4*cycles);
qr(:,1) = [l(2); l(3); l2(2); l2(3); l(2); l(3); l(2); l(3)];

count = 1;
for cycle=1:cycles % how many cycles of gait
    
    for legs=[2,4] % leg sequence

        for i=1:npoints - 1 % going through trajectory points

            if legs==4 %FL - leg 1
                Tbase = global_leg1HTM(straight_pose, 0.0, B(:, count), b, 0.0, L, r);
                P_d_w = Tbase * P_d_l;
                stance_w = Tbase * stance;

                stance_w = stance_w(1:3,1);
                P_d_w_1 = P_d_w(1:3,:);
            end
            if legs ==2 % BL - leg 2
                Tbase = global_leg2HTM(straight_pose, 0.0, B(:, count), b, 1.0, L, r);
                P_d_w = Tbase * P_d_l;
                stance_w = Tbase * stance;

                stance_w = stance_w(1:3,1);
                P_d_w_2 = P_d_w(1:3,:);
            end
            if legs==2 % FR - leg 3
                flipy = eye(4);
                flipy(2,2) = -1;
                Tbase = global_leg3HTM(straight_pose, 0.0, B(:, count), b, 0.0, L, r);
                P_d_w = Tbase * flipy * P_d_r;
                stance_w = Tbase * flipy * stance;

                stance_w = stance_w(1:3,1);
                P_d_w_3 = P_d_w(1:3,:);
            end
            if legs==4 %BR - leg 4
                flipy = eye(4);
                flipy(2,2) = -1;
                Tbase = global_leg4HTM(straight_pose, 0.0, B(:, count), b, 1.0, L, r);
                P_d_w = Tbase * flipy * P_d_r;
                stance_w = Tbase * flipy * stance;

                stance_w = stance_w(1:3,1);
                P_d_w_4 = P_d_w(1:3,:);
            end
            
            wf_x(:,count) = P_d_w(1:3,i);
            % T = global_COG(l, xi, B, b, xi_b, L, r);

            jc1 = Jacobian_leg1(qr(1:2, count), 1, B(:, count), b, 0.0, L, r);
            jc2 = Jacobian_leg2(qr(3:4, count), 1, B(:, count), b, 1.0, L, r);
            jc3 = Jacobian_leg3(qr(5:6, count), 1, B(:, count), b, 0.0, L, r);
            jc4 = Jacobian_leg4(qr(7:8, count), 1, B(:, count), b, 1.0, L, r);
            
            % constraint and unconstraint legs
            if legs==2 % leg 2 and leg 3 are swinging
                JC = [jc1; jc4];
                J = [jc2; jc3];

                dp1 = P_d_w_2(:,i+1) - P_d_w_2(:,i);
                dp2 = P_d_w_3(:,i+1) - P_d_w_3(:,i);
                % need to maintain the order (2 - 3)
            elseif legs == 4 % leg 1 and leg 4 are swinging
                JC = [jc2; jc3];
                J = [jc1; jc4];

                dp1 = P_d_w_1(:,i+1) - P_d_w_1(:,i);
                dp2 = P_d_w_4(:,i+1) - P_d_w_4(:,i);
                % need to maintain the order (1 - 4)
            end
            Jcog = Jacobian_cog(qr(1:8), 1, B(:, count), b, 1.0, L, r);

            
            dp = [dp1; dp2];
            JJ = [JC;J;Jcog];
            W = diag([1000*ones(k, 1);100*ones(swing_legs,1); 10*ones(3,1)]);

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

animateQuadrupedFast(qr, B, b, L, r, rBody)
% animateQuadrupedFastToVideo(qr, B, b, L, r, rBody, "walking_v5");