function y = gait(u)
%#codegen

time = u(1);
Bsen = u(2:7);
qr_len = u(8:15);
contact = u(16);


% -------------------- Persistents --------------------
persistent L r k T dt ...
           count legseq legs ...
           b straight_pose dp_cog ...
           isInitialized isTouched ...
           npoints stepLen_l stepLen_r lift ...
           qr_initial qr_next qr_dot

% -------------------- Initialization (single latch) --------------------
if isempty(isInitialized)

    % robot/motion parameters
    L  = 0.3175;
    r  = 0.013;
    k  = 9;
    T  = 1.0;
    dt = 0.001;
    qr_dot = zeros([8,1]);
    qr_next = zeros([8,1]);

    count  = 1;
    legseq = int32([2, 1, 4, 3]);   % int helps codegen indexing clarity
    legs   = int32(2);

    b             = -2e-8 * ones(2,1);
    straight_pose =  1e-8 * ones(2,1);
    dp_cog        = [1e-4; 0; 0];

    npoints   = int32(round(T/dt));
    stepLen_l = 0.04;
    stepLen_r = 0.04;
    lift      = 0.01;

    % initial swing sample (t=0)
    gait_l = swingTrajectory_timeDependant_simulink(stepLen_l, lift, r, L, T, [0.0, 0.0]);

    % initial body height from gait
    H = gait_l(1,1);
    Bsen(:,1) = [0;0;H; 0;0;0];

    % initial actuator lengths (precomputed constants you provided)
    qr_len(:,1) = [0.009230935630945;
               0.009230935630945;
               0.007570474340197;
               0.010946512434626;
               0.009230935630945;
               0.009230935630945;
               0.009230935630945;
               0.009230935630945];

    qr_initial   = qr_len(:,1);     % store as persistent (8x1)
    isTouched    = false;
    isInitialized = true;
    contact = 0;
end

% -------------------- Contact latch --------------------
if (contact == 1) && (~isTouched)
    isTouched = true;
end

% -------------------- Outputs defaults --------------------
% Always define outputs (codegen safe)
FL = zeros(2,1);
BL = zeros(2,1);
FR = zeros(2,1);
BR = zeros(2,1);
Bdy = zeros(2,1);

% -------------------- Main logic --------------------
if isTouched

    % local fixed-size buffers
    P_d_w = zeros(3,2);
    dp    = zeros(3,1);

    % Desired swing in leg-local frame (4x2 HTM points)
    if legs==1
        P_d_l = swingTrajectory_timeDependant_simulink(stepLen_l, lift, r, L, T, [time, time+dt]);
        Tbase = global_leg1HTM(straight_pose, 0.0, Bsen, b, 0.0, L, r);
        Pw = Tbase * P_d_l;
        P_d_w = Pw(1:3,:);
    elseif legs==2
        P_d_l = swingTrajectory_timeDependant_simulink(stepLen_l, lift, r, L, T, [time, time+dt]);
        Tbase = global_leg2HTM(straight_pose, 0.0, Bsen, b, 1.0, L, r);
        Pw = Tbase * P_d_l;
        P_d_w = Pw(1:3,:);
    elseif legs==3
        P_d_r = swingTrajectory_timeDependant_simulink(stepLen_r, lift, r, L, T, [time, time+dt]);
        flipy = eye(4);
        flipy(2,2) = -1;
        Tbase = global_leg3HTM(straight_pose, 0.0, Bsen, b, 0.0, L, r);
        Pw = Tbase * flipy * P_d_r;
        P_d_w = Pw(1:3,:);
    else % legs==4
        P_d_r = swingTrajectory_timeDependant_simulink(stepLen_r, lift, r, L, T, [time, time+dt]);
        flipy = eye(4);
        flipy(2,2) = -1;
        Tbase = global_leg4HTM(straight_pose, 0.0, Bsen, b, 1.0, L, r);
        Pw = Tbase * flipy * P_d_r;
        P_d_w = Pw(1:3,:);
    end

    % Jacobians (assumed fixed sizes in your functions)
    jc1 = Jacobian_leg1(qr_len(1:2,1), 1, Bsen, b, 0.0, L, r);
    jc2 = Jacobian_leg2(qr_len(3:4,1), 1, Bsen, b, 1.0, L, r);
    jc3 = Jacobian_leg3(qr_len(5:6,1), 1, Bsen, b, 0.0, L, r);
    jc4 = Jacobian_leg4(qr_len(7:8,1), 1, Bsen, b, 1.0, L, r);

    % Select constrained set JC (9x14) and swing leg J (3x14)
    JC = zeros(k,14);
    J  = zeros(3,14);

    if legs==1
        JC = [jc2; jc3; jc4];
        J  = jc1;
    elseif legs==2
        JC = [jc1; jc3; jc4];
        J  = jc2;
    elseif legs==3
        JC = [jc1; jc2; jc4];
        J  = jc3;
    else % legs==4
        JC = [jc1; jc2; jc3];
        J  = jc4;
    end

    Jcog = Jacobian_cog(qr_len(1:8,1), 1, Bsen, b, 1.0, L, r);

    % delta position desired
    dp = P_d_w(:,2) - P_d_w(:,1);

    % Solve weighted least squares:
    % JJ is (k+3+3) x 14 = 15 x 14, W is 15x15
    JJ = zeros(k+6, 14);
    JJ(1:k,:)       = JC;
    JJ(k+1:k+3,:)   = J;
    JJ(k+4:k+6,:)   = Jcog;

    W = diag([1000*ones(k,1); 100*ones(3,1); 10*ones(3,1)]); % 15x15

    rhs = [zeros(k,1); dp; dp_cog]; % 15x1

    % Update qr (only first 8 variables from the 14-delta)
    delta14 = (W*JJ) \ (W*rhs);          % 14x1
    qr_next = qr_len(:,1) + delta14(1:8);
    qr_dot = qr_next - qr_len(:,1);
    qr_len(:,1) = qr_len(:,1) + delta14(1:8);    % 8x1
    Bsen(:, 1) = Bsen(:, 1) - ( JC(:,9:end) \ ( JC(:, 1:8) * (qr_dot)));

    % Step the gait sequence
    count = count + 1;

    leg_idx = mod(floor((double(count)-1)/double(npoints)), numel(legseq)) + 1;
    legs    = legseq(leg_idx);

    % outputs
    FL = length2config(qr_len(1:2,1), r);
    BL = length2config(qr_len(3:4,1), r);
    FR = length2config(qr_len(5:6,1), r);
    BR = length2config(qr_len(7:8,1), r);
    Bdy = length2config(b, r);

else
    % Not touched yet: hold initial pose
    FL = length2config(qr_initial(1:2,1), r);
    BL = length2config(qr_initial(3:4,1), r);
    FR = length2config(qr_initial(5:6,1), r);
    BR = length2config(qr_initial(7:8,1), r);
    Bdy = length2config(b, r);
    qr_len(:,1) = qr_initial; % keep qr consistent
end

y = [FL; BL; FR; BR; Bdy; qr_len; Bsen];

end
