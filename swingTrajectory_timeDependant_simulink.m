function gait = swingTrajectory_timeDependant_simulink(stepLen, lift, r, L, T, t)
%#codegen
% ASSUMPTION: t is always 1x2.
% OUTPUT: gait is always 4x2: [X; Y; Z; HTM] at the two times.

% Fixed-size outputs
X   = zeros(1,2);
Y   = zeros(1,2);
Z   = zeros(1,2);
HTM = ones(1,2);

% Parameters
x0 = 0.202;
y0 = stepLen/2;

% Compute for each of the two time instants
for i = 1:2
    ti = t(i);

    % tau in [0,1) with special handling at exact multiples of T:
    % original intent: if mod(t,T)==0 and t~=0, set tau=1 (end of cycle)
    m = mod(ti, T);
    tau = m / T;

    % Robust "is multiple of T" check (avoid == on doubles)
    if (abs(m) < 1e-12) && (ti ~= 0)
        tau = 1.0;
    end

    % Smooth time scaling s(tau)
    s = 3*tau*tau - 2*tau*tau*tau;

    % Swing trajectory in task frame
    U  = lift * sin(pi*s);     % lift profile
    Xi = x0 - U;               % encode lift into X as you did
    Yi = y0 - stepLen*s;       % forward progression

    X(i) = Xi;
    Y(i) = Yi;

    % Get Z from task2length (must be codegen-safe)
    [~, Zi] = task2length([Xi; Yi], r, L);
    Z(i) = Zi;
end

% Return 4x2 (fixed size)
gait = [X; Y; Z; HTM];

end
