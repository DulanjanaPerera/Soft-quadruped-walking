function J = pressure2config_Jacobian(p)
% J: 2x3 finite double, robust to singular/near-singular inputs.

% ---- constants ----
A = pi*(0.013/2)^2;
K = 0.22991; %0.24991
r = 0.0130;

persistent J_prev is_init
if isempty(is_init)
    J_prev = zeros(2,3);   % initial fallback only for the very first call
    is_init = true;
end

% ---- input hygiene ----
p = double(p(:));
if numel(p) ~= 3 || any(~isfinite(p))
    J = J_prev;
    return;
end
p1 = p(1); p2 = p(2); p3 = p(3);

% ---- numerically safe helpers ----
EPS_DEN  = 1e-6;     % min magnitude for denominators (tune if needed)
EPS_SQRT = 1e-6;     % min value under sqrt (tune if needed)

den1 = 2*p2^2 + (-2*p1 - 2*p3)*p2 + 2*p1^2 - 2*p3*p1 + 2*p3^2;
den2 = 2*p1^2 + (-2*p2 - 2*p3)*p1 + 2*p2^2 - 2*p2*p3 + 2*p3^2;

EPS_QUAD = 1e-8;
% push denominators away from zero but preserve sign
if abs(den1) < EPS_DEN
    den1 = EPS_DEN * (2*(den1>=0)-1);
end
if abs(den2) < EPS_DEN
    den2 = EPS_DEN * (2*(den2>=0)-1);
end

quad = (p1^2 + (-p2 - p3)*p1 + p2^2 - p2*p3 + p3^2);

% Near straight configuration: hold previous J
if quad < EPS_QUAD
    J = J_prev;
    return;
end

phiarg = (A^2 * r^2) * quad;

% clamp sqrt argument to be nonnegative and not too small
phiarg = max(phiarg, EPS_SQRT);

% ---- rows of Jacobian ----
row1 = [ ...
   -sqrt(3) * (p2 - p3) / den1, ...
    sqrt(3) * (-p3 + p1) / den2, ...
   -sqrt(3) * (-p2 + p1) / den2 ];

inv_sqrt = 1 / sqrt(phiarg);
c = (A^2 * r^2) * inv_sqrt / (2*K);

row2 = [ ...
    c * (-p2 - p3 + 2*p1), ...
   -c * ( p1 - 2*p2 + p3), ...
   -c * ( p1 + p2 - 2*p3) ];

J_new = [row1; row2];


if any(~isfinite(J_new), 'all')
    J = J_prev;
    disp("NaN")
    return;
end

% ---- accept and store valid Jacobian ----
J_prev = J_new;
J = J_prev;

end
