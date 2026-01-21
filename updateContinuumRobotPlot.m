function updateContinuumRobotPlot(h, qLegs, B)
% updateContinuumRobotPlot
% Updates existing surfaces (fast) for body + 4 legs.

% --- Update body ---
Tb = globalBodyHTM(B);  % if you do not want bodyHTM, see note below.

switch lower(h.bodyAx)
    case 'x', dir_b = [1;0;0];
    case 'y', dir_b = [0;1;0];
    case 'z', dir_b = [0;0;1];
    otherwise, error('BodyAxis must be x,y,z.');
end
dir_b = h.bodySg * dir_b;

p0_b = [0;0;0];
p1_b = dir_b * h.L;

p0_g = Tb(1:3,1:3)*p0_b + Tb(1:3,4);
p1_g = Tb(1:3,1:3)*p1_b + Tb(1:3,4);

setTubeSegment(h, h.bodySurf, p0_g, p1_g, h.rBody);

% --- Update legs ---
xiVec = linspace(0,1,h.nXi);
legFns = {@global_leg1HTM, @global_leg2HTM, @global_leg3HTM, @global_leg4HTM};

for iLeg = 1:4
    qi = qLegs{iLeg};

    % Sample backbone points in WORLD
    P = zeros(h.nXi,3);
    for k = 1:h.nXi
        xi = xiVec(k);
        T  = legFns{iLeg}(qi, xi, B, h.L, h.rLeg);
        P(k,:) = T(1:3,4).';
    end

    % Update each tube segment
    hs = h.legSurf{iLeg};
    for k = 1:h.nSegLeg
        p0 = P(k,:).';
        p1 = P(k+1,:).';
        setTubeSegment(h, hs(k), p0, p1, h.rLeg);
    end
end

drawnow limitrate nocallbacks;
end

% -------------------------------------------------------------------------
function setTubeSegment(h, hSurf, p0, p1, rTube)
% Update a tube segment surface from p0 to p1 with radius rTube.

p0 = p0(:); p1 = p1(:);
v  = p1 - p0;
Lseg = norm(v);

if Lseg < 1e-9
    % Collapse segment
    set(hSurf, 'XData', 0*h.XC, 'YData', 0*h.YC, 'ZData', 0*h.ZC);
    return;
end

vhat = v / Lseg;

% Unit cylinder mesh scaled
XC = h.XC * rTube;
YC = h.YC * rTube;
ZC = h.ZC * Lseg;

% Rotation mapping z-axis to vhat
R = rotFromZ(vhat);

pts = R * [XC(:)'; YC(:)'; ZC(:)'];
X = reshape(pts(1,:), size(XC)) + p0(1);
Y = reshape(pts(2,:), size(YC)) + p0(2);
Z = reshape(pts(3,:), size(ZC)) + p0(3);

set(hSurf, 'XData', X, 'YData', Y, 'ZData', Z);
end

% -------------------------------------------------------------------------
function R = rotFromZ(vhat)
z = [0;0;1];
vhat = vhat(:);

c = dot(z, vhat);
if c > 1-1e-12
    R = eye(3);
    return;
elseif c < -1+1e-12
    R = [1 0 0; 0 -1 0; 0 0 -1];
    return;
end

axis = cross(z, vhat);
s = norm(axis);
axis = axis / s;

K = [    0    -axis(3)  axis(2);
      axis(3)     0    -axis(1);
     -axis(2)  axis(1)     0   ];
R = eye(3) + K*s + K*K*(1-c);
end
