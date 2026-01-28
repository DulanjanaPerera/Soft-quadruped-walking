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
hold on; grid on; axis equal;
view([1,90]); 
rotate3d 'on' ;
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-0.4 0.5]); ylim([-0.3 0.3]); zlim([0 0.4]);

camlight headlight;
lighting gouraud;

[XC, YC, ZC] = cylinder(1, h.nSides);
h.XC = XC; h.YC = YC; h.ZC = ZC;

% Tube segments
h.nSegLeg  = h.nXi - 1;

% Body is now also segmented
h.nSegBody = h.nXi - 1;

% Create body segment surfaces (dummy initially)
h.bodySurf = gobjects(h.nSegBody, 1);
for k = 1:h.nSegBody
    h.bodySurf(k) = createTubeSurfDummy(h);
end

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


function hSurf = createTubeSurfDummy(h)
% Create a tube segment surface with placeholder data (will be overwritten).
X = zeros(size(h.XC));
Y = zeros(size(h.YC));
Z = zeros(size(h.ZC));
hSurf = surf(X, Y, Z, 'EdgeColor', 'none');
end