function h = initContinuumRobotPlot(L, rLeg, rBody, varargin)

p = inputParser;
p.addParameter('nXi', 50, @(x)isnumeric(x)&&isscalar(x)&&x>=5);
p.addParameter('nSides', 32, @(x)isnumeric(x)&&isscalar(x)&&x>=6);
p.addParameter('BodyAxis', 'x', @(s)ischar(s)||isstring(s));
p.addParameter('BodySign', -1, @(x)isnumeric(x)&&isscalar(x)&&(x==1||x==-1));
p.parse(varargin{:});

h.nXi    = p.Results.nXi;
h.nSides = p.Results.nSides;
h.bodyAx = char(p.Results.BodyAxis);
h.bodySg = p.Results.BodySign;

h.L = L;
h.rLeg = rLeg;
h.rBody = rBody;

figure(1); clf;
ax = axes; %#ok<NASGU>
hold on; grid on; axis equal;
view([1,90]);
rotate3d on;

xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-0.4 0.5]); ylim([-0.3 0.3]); zlim([0 0.4]);

camlight headlight;
lighting gouraud;

[XC, YC, ZC] = cylinder(1, h.nSides);
h.XC = XC; h.YC = YC; h.ZC = ZC;

h.nSegLeg  = h.nXi - 1;
h.nSegBody = h.nXi - 1;

% Body segment surfaces
h.bodySurf = gobjects(h.nSegBody, 1);
for k = 1:h.nSegBody
    h.bodySurf(k) = createTubeSurfDummy(h);
end

% Leg segment surfaces
h.legSurf = cell(4,1);
for iLeg = 1:4
    hs = gobjects(h.nSegLeg,1);
    for k = 1:h.nSegLeg
        hs(k) = createTubeSurfDummy(h);
    end
    h.legSurf{iLeg} = hs;
end

% -------- NEW: support polygon + feet + CoG graphics (created once) --------
h.supportPatch = patch('XData', nan, 'YData', nan, 'ZData', nan, ...
    'FaceAlpha', 0.15, 'EdgeColor', 'k', 'LineWidth', 1.5);

h.supportEdge = line(nan, nan, nan, 'LineWidth', 2);

h.footDotsAll     = scatter3(nan, nan, nan, 30, 'filled');  % all feet tips
h.footDotsContact = scatter3(nan, nan, nan, 50, 'filled');  % only contacts

h.cogDot3D   = scatter3(nan, nan, nan, 70, 'filled');
h.cogDotProj = scatter3(nan, nan, 0,  50, 'filled');
h.cogStem    = line(nan, nan, nan, 'LineStyle', '--', 'LineWidth', 1);

% defaults (can be overwritten outside)
h.contactZThresh = 0.005;
h.useRelativeGround = true;
h.groundPad = 0.002;

end

function hSurf = createTubeSurfDummy(h)
% Create a tube segment surface with placeholder data (will be overwritten).
X = zeros(size(h.XC));
Y = zeros(size(h.YC));
Z = zeros(size(h.ZC));
hSurf = surf(X, Y, Z, 'EdgeColor', 'none');
end
