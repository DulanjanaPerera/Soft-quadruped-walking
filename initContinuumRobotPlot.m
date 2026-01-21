function h = initContinuumRobotPlot(L, rLeg, rBody, varargin)
% initContinuumRobotPlot
% Creates the graphics objects ONCE and returns handles/geometry cache.
%
% Use with updateContinuumRobotPlot() for fast animation.

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

% --- Figure/axes setup (fixed axes) ---
figure(1); clf;
hold on; grid on; axis equal; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-0.4 0.5]); ylim([-0.3 0.3]); zlim([0 0.4]);

% lighting once
camlight headlight;
lighting gouraud;

% --- Precompute base cylinder mesh (unit cylinder along +Z) ---
[XC, YC, ZC] = cylinder(1, h.nSides);   % radius 1
h.XC = XC; h.YC = YC; h.ZC = ZC;

% Number of tube segments:
h.nSegLeg  = h.nXi - 1;
h.nSegBody = 1;

% Create body surface (dummy initially)
[h.bodySurf] = createTubeSurfDummy(h);

% Create leg segment surfaces (dummy initially)
h.legSurf = cell(4,1);
for iLeg = 1:4
    hs = gobjects(h.nSegLeg,1);
    for k = 1:h.nSegLeg
        hs(k) = createTubeSurfDummy(h);
    end
    h.legSurf{iLeg} = hs;
end

end

% -------------------------------------------------------------------------
function hSurf = createTubeSurfDummy(h)
% Create a tube segment surface with placeholder data (will be overwritten).
X = zeros(size(h.XC));
Y = zeros(size(h.YC));
Z = zeros(size(h.ZC));
hSurf = surf(X, Y, Z, 'EdgeColor', 'none');
end
