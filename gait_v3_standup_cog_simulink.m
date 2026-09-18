function y = gait_v3_standup_cog_simulink(u)
%#codegen
%GAIT_V3_STANDUP_COG_SIMULINK
% Simulink MATLAB Function block version of the current offline gait script.
%
% Main change from gait_v2:
%   1) No contact latch/contact-detection input.
%   2) State order is time based:
%        Phase 1: stand-up     0 <= t < standupTime
%        Phase 2: hold         standupTime <= t < standupTime + holdTime
%        Phase 3: walking      t >= standupTime + holdTime
%   3) Walking uses the current Figure-8 CoG regulation and weighted LS solve.
%
% INPUT u, 16x1:
%   u(1)      = current simulation time, time
%   u(2:7)    = previous/feedback body state Bsen, 6x1
%   u(8:15)   = previous/feedback actuator length state qr_len, 8x1
%   u(16)     = previous simulation time, prev_time
%
% OUTPUT y, 27x1:
%   y(1:2)    = FL configuration [theta; phi]
%   y(3:4)    = BL configuration [theta; phi]
%   y(5:6)    = FR configuration [theta; phi]
%   y(7:8)    = BR configuration [theta; phi]
%   y(9:10)   = body configuration [theta; phi]
%   y(11:18)  = qr_len, 8x1
%   y(19:24)  = Bsen, 6x1
%   y(25)     = time
%   y(26)     = swingLeg, 0 during stand-up/hold, otherwise one of [2 1 4 3]
%   y(27)     = phaseID, 1 stand-up, 2 hold, 3 walking
%
% Required external functions already in your project path:
%   swingTrajectory_timeDependant_simulink
%   config2length, config2task, length2config
%   global_leg1HTM, global_leg2HTM, global_leg3HTM, global_leg4HTM
%   Jacobian_leg1, Jacobian_leg2, Jacobian_leg3, Jacobian_leg4
%   Jacobian_cog

% -------------------- Inputs --------------------
time      = u(1);
Bsen      = u(2:7);
qr_len    = u(8:15);
prev_time = u(16);

% -------------------- Parameters --------------------
L  = 0.317475;
r  = 0.013;
k  = 9;

T = 1.0;              % swing duration for one leg
standupTime = 2.0;    % stand-up duration
holdTime    = 1.0;    % hold duration after standing

stepLen_l = 0.02;
stepLen_r = 0.02;
lift      = 0.01;

b             = -2e-8 * ones(2,1);
straight_pose =  1e-8 * ones(2,1);

% Numerical damping. Increase lambda_base first if body jump appears.
lambda_q    = 1e-8;
lambda_base = 1e-6;

% Codegen-safe output defaults.
FL  = zeros(2,1);
BL  = zeros(2,1);
FR  = zeros(2,1);
BR  = zeros(2,1);
Bdy = zeros(2,1);
swingLeg = 0.0;
phaseID  = 1.0;

% Avoid negative or nonmonotonic time issues during initialization/reset.
if time < 0.0
    time = 0.0;
end
if prev_time < 0.0
    prev_time = 0.0;
end
if prev_time > time
    prev_time = time;
end

walkStartTime = standupTime + holdTime;

% -------------------- Phase 1: stand-up --------------------
if time < standupTime
    phaseID = 1.0;
    [qr_len, Bsen] = localStandupPose(time, standupTime, r, L);
    swingLeg = 0.0;

% -------------------- Phase 2: hold --------------------
elseif time < walkStartTime
    phaseID = 2.0;
    [qr_len, Bsen] = localStandupPose(standupTime, standupTime, r, L);
    swingLeg = 0.0;

% -------------------- Phase 3: walking --------------------
else
    phaseID = 3.0;

    % If the feedback states entering the block are zero/invalid at the first
    % walking instant, initialize from the final standing pose.
    if localBadState(qr_len, Bsen)
        [qr_len, Bsen] = localStandupPose(standupTime, standupTime, r, L);
    end

    % Walking time coordinates.
    tWalk1 = time - walkStartTime;
    tWalk0 = prev_time - walkStartTime;
    if tWalk0 < 0.0
        tWalk0 = 0.0;
    end
    if tWalk1 < 0.0
        tWalk1 = 0.0;
    end
    if tWalk0 > tWalk1
        tWalk0 = tWalk1;
    end

    % Leg sequence: same order as offline script.
    legseq = int32([2; 1; 4; 3]);
    seg1 = floor(tWalk1 / T);
    seg0 = floor(tWalk0 / T);
    idx1 = int32(mod(seg1, 4.0) + 1.0);
    legs = legseq(idx1);
    swingLeg = double(legs);

    % Local time inside current swing interval.
    local1 = tWalk1 - seg1*T;
    if seg0 == seg1
        local0 = tWalk0 - seg0*T;
    else
        % New leg started between prev_time and time. Start this leg from 0.
        local0 = 0.0;
    end

    if local0 < 0.0
        local0 = 0.0;
    end
    if local1 < 0.0
        local1 = 0.0;
    end
    if local0 > T
        local0 = T;
    end
    if local1 > T
        local1 = T;
    end

    traj_time = [local0, local1];

    % ---------------- Desired swing-foot displacement ----------------
    P_d_w = zeros(3,2);

    if legs == 1
        P_d_l = swingTrajectory_timeDependant_simulink(stepLen_l, lift, r, L, T, traj_time);
        Tbase = global_leg1HTM(straight_pose, 0.0, Bsen, b, 0.0, L, r);
        Pw = Tbase * P_d_l;
        P_d_w = Pw(1:3,:);

    elseif legs == 2
        P_d_l = swingTrajectory_timeDependant_simulink(stepLen_l, lift, r, L, T, traj_time);
        Tbase = global_leg2HTM(straight_pose, 0.0, Bsen, b, 1.0, L, r);
        Pw = Tbase * P_d_l;
        P_d_w = Pw(1:3,:);

    elseif legs == 3
        P_d_r = swingTrajectory_timeDependant_simulink(stepLen_r, lift, r, L, T, traj_time);
        flipy = eye(4);
        flipy(2,2) = -1.0;
        Tbase = global_leg3HTM(straight_pose, 0.0, Bsen, b, 0.0, L, r);
        Pw = Tbase * flipy * P_d_r;
        P_d_w = Pw(1:3,:);

    else % legs == 4
        P_d_r = swingTrajectory_timeDependant_simulink(stepLen_r, lift, r, L, T, traj_time);
        flipy = eye(4);
        flipy(2,2) = -1.0;
        Tbase = global_leg4HTM(straight_pose, 0.0, Bsen, b, 1.0, L, r);
        Pw = Tbase * flipy * P_d_r;
        P_d_w = Pw(1:3,:);
    end

    dp = P_d_w(:,2) - P_d_w(:,1);

    % ---------------- Current foot positions for CoG reference ----------------
    T1 = global_leg1HTM(qr_len(1:2), 1.0, Bsen, b, 0.0, L, r);
    T2 = global_leg2HTM(qr_len(3:4), 1.0, Bsen, b, 1.0, L, r);
    T3 = global_leg3HTM(qr_len(5:6), 1.0, Bsen, b, 0.0, L, r);
    T4 = global_leg4HTM(qr_len(7:8), 1.0, Bsen, b, 1.0, L, r);

    footPos = [T1(1:3,4), T2(1:3,4), T3(1:3,4), T4(1:3,4)];

    COG_now  = localCogFigure8Time(tWalk0, T, footPos, stepLen_l);
    COG_next = localCogFigure8Time(tWalk1, T, footPos, stepLen_l);

    dtWalk = tWalk1 - tWalk0;
    if dtWalk < 0.0
        dtWalk = 0.0;
    end
    dp_cog_vel = [stepLen_l/(4.0*T) * dtWalk; 0.0; 0.0];
    dp_cog = (COG_next - COG_now) + dp_cog_vel;

    % ---------------- Jacobians ----------------
    jc1 = Jacobian_leg1(qr_len(1:2), 1.0, Bsen, b, 0.0, L, r);
    jc2 = Jacobian_leg2(qr_len(3:4), 1.0, Bsen, b, 1.0, L, r);
    jc3 = Jacobian_leg3(qr_len(5:6), 1.0, Bsen, b, 0.0, L, r);
    jc4 = Jacobian_leg4(qr_len(7:8), 1.0, Bsen, b, 1.0, L, r);

    JC = zeros(k,14);
    J  = zeros(3,14);

    if legs == 1
        JC = [jc2; jc3; jc4];
        J  = jc1;
    elseif legs == 2
        JC = [jc1; jc3; jc4];
        J  = jc2;
    elseif legs == 3
        JC = [jc1; jc2; jc4];
        J  = jc3;
    else % legs == 4
        JC = [jc1; jc2; jc3];
        J  = jc4;
    end

    Jcog = Jacobian_cog(qr_len(1:8), 1.0, Bsen, b, 1.0, L, r);

    % ---------------- Weighted least-squares update ----------------
    JJ = zeros(k+6,14);
    JJ(1:k,:)     = JC;
    JJ(k+1:k+3,:) = J;
    JJ(k+4:k+6,:) = Jcog;

    % Same priority structure as the offline current script:
    % stance constraints low/moderate, swing foot high, CoG soft.
    W = diag([10.0*ones(k,1); 1000.0*ones(3,1); 0.1; 0.1; 0.1]);
    rhs = [zeros(k,1); dp; dp_cog];

    delta14 = localDampedLS(W*JJ, W*rhs, lambda_q);
    dq = delta14(1:8);

    qr_len = qr_len + dq;

    % Base update from stance constraints. Damped LS is safer than raw
    % backslash when JC(:,9:end) is close to rank-deficient.
    A_base = JC(:,9:14);
    b_base = JC(:,1:8) * dq;
    dB = localDampedLS(A_base, b_base, lambda_base);
    Bsen = Bsen - dB;
end

% -------------------- Convert lengths to configuration outputs --------------------
FL  = length2config(qr_len(1:2), r);
BL  = length2config(qr_len(3:4), r);
FR  = length2config(qr_len(5:6), r);
BR  = length2config(qr_len(7:8), r);
Bdy = length2config(b, r);

% -------------------- Output vector --------------------
y = [FL; BL; FR; BR; Bdy; qr_len(1:8); Bsen; time; swingLeg; phaseID];

end

% ========================================================================
% Local helper functions
% ========================================================================

function [qr, B] = localStandupPose(time, standupTime, r, L)
% Smooth stand-up pose using the same cubic smoothstep as the offline script.
if standupTime <= 0.0
    tau = 1.0;
else
    tau = time / standupTime;
end
if tau < 0.0
    tau = 0.0;
elseif tau > 1.0
    tau = 1.0;
end
s = 3.0*tau*tau - 2.0*tau*tau*tau;

theta = 0.0;
phi   = s*pi/2.0;

l = config2length(theta, phi, r);
l2 = l(2);
l3 = l(3);
p = config2task(theta, phi, L);

qr = [l2; l3; l2; l3; l2; l3; l2; l3];
B  = [0.0; 0.0; p(1); 0.0; 0.0; 0.0];
end

function bad = localBadState(qr, B)
% Detect uninitialized feedback memory states.
bad = false;
if any(~isfinite(qr)) || any(~isfinite(B))
    bad = true;
end
if norm(qr) < 1e-12 && norm(B) < 1e-12
    bad = true;
end
end

function COG_d = localCogFigure8Time(tWalk, T, footPos, stepLen)
% Time-based equivalent of cogFigure8(k,npoints,...).
% One full Figure-8 CoG cycle spans four swing phases, i.e., 4*T seconds.
p1 = footPos(:,1);
p2 = footPos(:,2);
p3 = footPos(:,3);
p4 = footPos(:,4);

xc = 0.25 * (p1 + p2 + p3 + p4);

e_fb = [1.0; 0.0; 0.0];
e_lr = [0.0; 1.0; 0.0];

alpha_lr = 0.2;
alpha_fb = 0.2;
A_lr = alpha_lr * stepLen;
A_fb = alpha_fb * stepLen;

period = 4.0*T;
if period <= 0.0
    phase = 0.0;
else
    tmod = tWalk - floor(tWalk/period)*period;
    phase = 2.0*pi*tmod/period;
end

COG_d = xc ...
      + A_lr * e_lr * sin(phase - pi/2.0) ...
      + A_fb * e_fb * sin(2.0*phase - pi);
COG_d(3) = 0.0;
end

function x = localDampedLS(A, b, lambda)
% Damped least-squares solution for codegen-safe Simulink execution.
% Solves min ||A*x-b||^2 + lambda^2 ||x||^2.
[m,n] = size(A); %#ok<ASGLU>
AtA = A.'*A;
Atb = A.'*b;
x = (AtA + (lambda*lambda)*eye(n)) \ Atb;
end
