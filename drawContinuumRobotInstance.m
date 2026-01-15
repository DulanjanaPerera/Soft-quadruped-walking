function h = drawContinuumRobotInstance(qLegs, B, L, rLeg, rBody, varargin)
% drawContinuumRobotInstance
% Draws a quadruped with 4 continuum legs (as tubes along backbone) and a straight
% cylindrical body of length L.
%
% REQUIRED INPUTS
%   qLegs : cell array {q1,q2,q3,q4} where qi is the continuum parameter vector
%           used by global_leg{i}HTM (e.g., 2x1 for fixed-backbone CC modal).
%   B     : global body parameter vector (6x1) for globalBodyHTM(B)
%   L     : body length (and typically leg nominal length, if you use same L)
%   rLeg  : leg tube radius (m)
%   rBody : body tube radius (m)
%
% OPTIONAL NAME-VALUE
%   'nXi'      : number of xi samples along each leg (default 25)
%   'nSides'   : tube sides (default 14)
%   'BodyAxis' : axis of body cylinder in body frame: 'x','y','z' (default 'y')
%   'BodySign' : +1 or -1 direction along that axis (default -1)
%   'Hold'     : true/false; if false, clears axes (default false)
%
% OUTPUT
%   h : struct of graphics handles

p = inputParser;
p.addParameter('nXi', 25, @(x)isnumeric(x)&&isscalar(x)&&x>=5);
p.addParameter('nSides', 14, @(x)isnumeric(x)&&isscalar(x)&&x>=6);
p.addParameter('BodyAxis', 'y', @(s)ischar(s)||isstring(s));
p.addParameter('BodySign', -1, @(x)isnumeric(x)&&isscalar(x)&&(x==1||x==-1));
p.addParameter('Hold', false, @(x)islogical(x)&&isscalar(x));
p.parse(varargin{:});

nXi    = p.Results.nXi;
nSides = p.Results.nSides;
bodyAx = char(p.Results.BodyAxis);
bodySg = p.Results.BodySign;
doHold = p.Results.Hold;

if ~doHold
    cla; hold on;
else
    hold on;
end
axis equal; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);

% --- 1) Draw body as a straight tube ---
Tb = globalBodyHTM(B); % 4x4 HTM, body frame at front junction (your convention)

% Choose body direction in BODY frame (edit here if needed)
switch lower(bodyAx)
    case 'x', dir_b = [1;0;0];
    case 'y', dir_b = [0;1;0];
    case 'z', dir_b = [0;0;1];
    otherwise, error('BodyAxis must be ''x'',''y'', or ''z''.');
end
dir_b = bodySg * dir_b;

% Body endpoints in body frame: from origin to length L along dir_b
p0_b = [0;0;0];
p1_b = dir_b * L;

% Transform endpoints to global
p0_g = Tb(1:3,1:3)*p0_b + Tb(1:3,4);
p1_g = Tb(1:3,1:3)*p1_b + Tb(1:3,4);

h.body = drawTubeSegment(p0_g, p1_g, rBody, nSides);

% --- 2) Draw each continuum leg as a tube along sampled backbone ---
if ~iscell(qLegs) || numel(qLegs) ~= 4
    error('qLegs must be a cell array {q1,q2,q3,q4}.');
end

xiVec = linspace(0,1,nXi);

legFns = {@global_leg1HTM, @global_leg2HTM, @global_leg3HTM, @global_leg4HTM};
h.legs = gobjects(4,1);

for i = 1:4
    qi = qLegs{i};

    % Sample backbone positions in GLOBAL frame by calling your HTM
    P = zeros(nXi,3);
    for k = 1:nXi
        xi = xiVec(k);
        T  = legFns{i}(qi, xi, B, L, rLeg);    % uses your function signature
        P(k,:) = T(1:3,4).';
    end

    % Draw tube as short segments between consecutive sampled points
    hg = gobjects(nXi-1,1);
    for k = 1:(nXi-1)
        hg(k) = drawTubeSegment(P(k,:).', P(k+1,:).', rLeg, nSides);
    end
    h.legs(i) = hg(1); % representative handle
end

% --- Visual styling ---
camlight headlight; lighting gouraud;

end

% ============================================================
function hSurf = drawTubeSegment(p0, p1, r, nSides)
% Draw a cylinder segment from p0 to p1 with radius r.

p0 = p0(:); p1 = p1(:);
v  = p1 - p0;
Lseg = norm(v);
if Lseg < 1e-9
    hSurf = gobjects(1);
    return;
end
vhat = v / Lseg;

% Base cylinder along +Z from z=0..Lseg
[XC, YC, ZC] = cylinder(r, nSides);
ZC = ZC * Lseg;

% Build rotation that maps z-axis to vhat
R = rotFromZ(vhat);

% Apply transform: [R p0; 0 0 0 1]
pts = R * [XC(:)'; YC(:)'; ZC(:)'];
X = reshape(pts(1,:), size(XC)) + p0(1);
Y = reshape(pts(2,:), size(YC)) + p0(2);
Z = reshape(pts(3,:), size(ZC)) + p0(3);

hSurf = surf(X, Y, Z, 'EdgeColor', 'none');
end

% ============================================================
function R = rotFromZ(vhat)
% Return rotation matrix R such that R*[0;0;1] = vhat.
z = [0;0;1];
vhat = vhat(:);

c = dot(z, vhat);
if c > 1-1e-12
    R = eye(3);
    return;
elseif c < -1+1e-12
    % 180 deg rotation about x-axis (or any axis orthogonal to z)
    R = [1 0 0; 0 -1 0; 0 0 -1];
    return;
end

axis = cross(z, vhat);
s = norm(axis);
axis = axis / s;

% Rodrigues
K = [    0    -axis(3)  axis(2);
      axis(3)     0    -axis(1);
     -axis(2)  axis(1)     0   ];
R = eye(3) + K*s + K*K*(1-c);
end
