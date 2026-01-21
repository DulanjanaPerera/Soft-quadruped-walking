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

stepLen = 0.015;     % step length
lift    = 0.01;
[X, Y, Z] = swingTrajectory(stepLen, lift);

npoints = length(X);
P_d = [X; Y; Z; ones(1,npoints)];

L = 0.278;        % m
r  = 0.013;   % m
rBody = 0.012;   % m
cycles = 5;
k = 9;

H = X(1); % the leg's X is worldframe Z. So the standing position.

pX = zeros(1,npoints*4*cycles);
B = zeros(6,npoints*4*cycles);
B(:,1) = [0;0;H; 0;0;0];  % [x y z roll pitch yaw] or your convention

[l,~] = task2length([H; 0], r, L);
[l2,~] = task2length([X(1); Y(1)], r, L);

% initial q_legs
% leg 2 is going to swing
% q1 = [l(2); l(3)];
% q2 = [l2(2); l2(3)];
% q3 = [l(2); l(3)];
% q4 = [l(2); l(3)];

qr = zeros(8, npoints*4*cycles);
qr(:,1) = [l(2); l(3); l2(2); l2(3); l(2); l(3); l(2); l(3)];
count = 1;
for cycle=1:cycles % how many cycles of gait
    
    for legs=[2,1,4,3] % leg sequence

        for i=1:npoints - 1 % going through trajectory points

            % Coordinate rotation: leg frame -> world frame
            % Leg X -> World Z, Leg Y -> World X, Leg Z -> World Y
            R_leg_to_world = [0 1 0 0;   % world X from leg Y
                              0 0 1 0;   % world Y from leg Z
                              1 0 0 0;   % world Z from leg X
                              0 0 0 1];

            if legs==1
                Tbase = global_leg1HTM([0.0;0.0], 0.0, B(:, count), L, r);
                % transform to world frame with coordinate rotation
                P_d_w = Tbase * R_leg_to_world * P_d;
                P_d_w = P_d_w(1:3,:);
            elseif legs ==2
                Tbase = global_leg2HTM([0.0;0.0], 0.0, B(:, count), L, r);
                % transform to world frame with coordinate rotation
                P_d_w = Tbase * R_leg_to_world * P_d;
                P_d_w = P_d_w(1:3,:);
            elseif legs==3
                flipy = eye(4);
                flipy(2,2) = -1;
                Tbase = global_leg3HTM([0.0;0.0], 0.0, B(:, count), L, r);
                % transform to world frame with coordinate rotation and Y flip
                P_d_w = Tbase * flipy * R_leg_to_world * P_d;
                P_d_w = P_d_w(1:3,:);
            elseif legs==4
                flipy = eye(4);
                flipy(2,2) = -1;
                Tbase = global_leg4HTM([0.0;0.0], 0.0, B(:, count), L, r);
                % transform to world frame with coordinate rotation and Y flip
                P_d_w = Tbase * flipy * R_leg_to_world * P_d;
                P_d_w = P_d_w(1:3,:);
            end

            jc1 = Jacobian_leg1(qr(1:2, count), 1, B(:, count), L, r);
            jc2 = Jacobian_leg2(qr(3:4, count), 1, B(:, count), L, r);
            jc3 = Jacobian_leg3(qr(5:6, count), 1, B(:, count), L, r);
            jc4 = Jacobian_leg4(qr(7:8, count), 1, B(:, count), L, r);

            % JJ = [jc1; jc2; jc3; jc4];
            % p_des = zeros(12,1);
            % p_des(3*(legs-1)+1:3*legs,1) = P_d_w(:,i+1) - P_d_w(:,i);
            
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


            dp = P_d_w(:,i+1) - P_d_w(:,i);
            % dp(1) = dp(1) + 0.001;   % REMOVED: coordinate transformation now handles this properly

            % dq = pinv([JC; J]) * ([zeros(k,1); P_d_w(:,i+1)-P_d_w(:,i)] );
            JJ = [JC;J];

            % dq = pinv(JJ) * ([zeros(k,1); dp]);   % 14x1
            % qr(:,count+1) = qr(:,count) + dq(1:8);
            % B(:,count+1)  = B(:,count)  + dq(9:14);

            % dq = JJ \ p_des;
            % qr(:, count+1) = qr(:, count) + [eye(8), zeros(8,6)] * pinv([JC; J]) * ([zeros(k,1); P_d_w(:,i+1)-P_d_w(:,i)] );
            % B(:, count+1) = B(:, count) - pinv(JC(:,9:end)) * JC(:, 1:8) * (qr(:, count+1) - qr(:, count));
            qr(:, count+1) = qr(:, count) + [eye(8), zeros(8,6)] * ( JJ \ ([zeros(k,1); dp]) );
            B(:, count+1) = B(:, count) - ( JC(:,9:end) \ ( JC(:, 1:8) * (qr(:, count+1) - qr(:, count)) ) );
           
            % qr(:, count+1) = qr(:, count) + [eye(8), zeros(8,6)] * ( JJ \ (p_des) );
            % B(:, count+1) = B(:, count) - ( JC(:,9:end) \ ( JC(:, 1:8) * (qr(:, count+1) - qr(:, count)) ) );

            count = count + 1;
        end
    end
end

vec = 1:1:count;

figure(2)
plot((1:1:count),B(1,1:count), (1:1:count),B(2,1:count), (1:1:count),B(3,1:count));
grid on
axis tight
xlabel 'interations'
ylabel 'Cartesian'
legend 'X' 'Y' 'Z'

% figure(3)
% plot((1:1:count),B(1,1:count), (1:1:count),B(2,1:count), (1:1:count),B(3,1:count));
% grid on
% axis tight
% xlabel 'interations'
% ylabel 'Cartesian'
% legend 'X' 'Y' 'Z'



animateQuadrupedFast(qr, B, L, r, rBody)
% animateQuadrupedFastToVideo(qr, B, L, r, rBody, "walking_v3");