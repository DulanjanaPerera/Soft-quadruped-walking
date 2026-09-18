function updateContinuumRobotPlot(h, qLegs, B, b, lAll)
% Updates surfaces for flex body + 4 legs + support polygon + CoG.

xiVec = linspace(0,1,h.nXi);

% --- Update FLEX BODY ---
Pb = zeros(h.nXi, 3);
for k = 1:h.nXi
    xi_b = xiVec(k);
    Tb = global_bodyHTM(B, b, xi_b, h.L, h.rLeg);
    Pb(k,:) = Tb(1:3,4).';
end
for k = 1:h.nSegBody
    setTubeSegment(h, h.bodySurf(k), Pb(k,:).', Pb(k+1,:).', h.rBody);
end

% --- Update LEGS (and also collect tip positions) ---
legFns = {@global_leg1HTM, @global_leg2HTM, @global_leg3HTM, @global_leg4HTM};
xi_b_attach = [0.0; 1.0; 0.0; 1.0];

pFeet = zeros(3,4);

for iLeg = 1:4
    qi = qLegs{iLeg};

    P = zeros(h.nXi, 3);
    for k = 1:h.nXi
        xi = xiVec(k);
        T = legFns{iLeg}(qi, xi, B, b, xi_b_attach(iLeg), h.L, h.rLeg);
        P(k,:) = T(1:3,4).';
    end

    hs = h.legSurf{iLeg};
    for k = 1:h.nSegLeg
        setTubeSegment(h, hs(k), P(k,:).', P(k+1,:).', h.rLeg);
    end

    % foot tip at xi = 1
    pFeet(:,iLeg) = P(end,:).';
end

% Plot all foot tips
set(h.footDotsAll, 'XData', pFeet(1,:), 'YData', pFeet(2,:), 'ZData', pFeet(3,:));

% --- Contact detection ---
if h.useRelativeGround
    zGround = min(pFeet(3,:));   % estimate ground at lowest foot
    zThresh = zGround + h.groundPad;
else
    zThresh = h.contactZThresh;  % absolute threshold above z=0
end

inContact = (pFeet(3,:) <= zThresh);
idx = find(inContact);

% Plot only contacting feet
set(h.footDotsContact, 'XData', pFeet(1,idx), 'YData', pFeet(2,idx), 'ZData', pFeet(3,idx));

% --- Support polygon on XY plane ---
if numel(idx) >= 3
    XY = pFeet(1:2, idx).';  % Nx2

    % Order vertices robustly by convex hull in XY
    K = convhull(XY(:,1), XY(:,2));
    K(end) = [];

    polyXY = XY(K,:);
    polyZ  = zeros(size(polyXY,1),1);

    set(h.supportPatch, 'XData', polyXY(:,1), 'YData', polyXY(:,2), 'ZData', polyZ);
    set(h.supportEdge,  'XData', [polyXY(:,1); polyXY(1,1)], ...
                        'YData', [polyXY(:,2); polyXY(1,2)], ...
                        'ZData', zeros(size(polyXY,1)+1,1));
else
    % hide if <3 contacts
    set(h.supportPatch, 'XData', nan, 'YData', nan, 'ZData', nan);
    set(h.supportEdge,  'XData', nan, 'YData', nan, 'ZData', nan);
end

% --- CoG in 3D + projection ---
% Your function returns HTM at CoG (4x4). Use translation.
Tcog = global_COG(lAll, 0, B, b, 1, h.L, h.rLeg);  % xi_b not needed for CoG; set 0
pCoG = Tcog(1:3,4);

set(h.cogDot3D,   'XData', pCoG(1), 'YData', pCoG(2), 'ZData', pCoG(3));
set(h.cogDotProj, 'XData', pCoG(1), 'YData', pCoG(2), 'ZData', 0);
set(h.cogStem,    'XData', [pCoG(1) pCoG(1)], ...
                  'YData', [pCoG(2) pCoG(2)], ...
                  'ZData', [0 pCoG(3)]);

% NOTE: do NOT use nocallbacks if you want to rotate interactively
% drawnow is handled in the animation loop
end

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
