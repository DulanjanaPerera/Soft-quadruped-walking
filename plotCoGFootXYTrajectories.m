function data = plotCoGFootXYTrajectories(qr, B, b, L, rLeg, varargin)
% plotCoGFootXYTrajectories
%
% Plots XY trajectories of:
%   1) CoG projection
%   2) Foot contact trajectories
%   3) Optional support polygons
%
% Inputs:
%   qr   : 8xN leg length trajectory
%   B    : 6xN body trajectory [X;Y;Z;roll;pitch;yaw]
%   b    : 2x1 or 2xN body length changes
%   L    : module length
%   rLeg : leg radius
%
% Optional name-value inputs:
%   'SwingLeg'          : 1xN vector. 0 means no swing/all stance.
%                         1,2,3,4 means that leg is swinging.
%                         If provided, this is used for contact detection.
%
%   'ContactZThresh'    : absolute Z threshold for contact detection.
%                         Default = 0.005 m.
%
%   'UseRelativeGround' : if true, ground is estimated from lowest foot.
%                         Default = false.
%
%   'GroundPad'         : extra margin above estimated ground.
%                         Default = 0.002 m.
%
%   'PlotSupport'       : true/false. Plot support polygons.
%                         Default = true.
%
%   'SupportEvery'      : plot support polygon every N samples.
%                         Default = 10.
%
%   'PlotSwingProjection' : true/false. Plot all foot XY projections,
%                           including swing phase.
%                           Default = false.
%
% Output:
%   data.pFeet      : 3x4xN foot positions
%   data.pCoG       : 3xN CoG positions
%   data.inContact  : 4xN contact flags
%   data.footXYPlot : 4xN struct-like arrays with NaN gaps

% -----------------------------
% Parse optional inputs
% -----------------------------
p = inputParser;
p.addParameter('SwingLeg', []);
p.addParameter('ContactZThresh', 0.005);
p.addParameter('UseRelativeGround', false);
p.addParameter('GroundPad', 0.002);
p.addParameter('PlotSupport', false);
p.addParameter('SupportEvery', 10);
p.addParameter('PlotSwingProjection', false);
p.parse(varargin{:});

swingLeg          = p.Results.SwingLeg;
contactZThresh    = p.Results.ContactZThresh;
useRelativeGround = p.Results.UseRelativeGround;
groundPad         = p.Results.GroundPad;
plotSupport       = p.Results.PlotSupport;
supportEvery      = p.Results.SupportEvery;
plotSwingProj     = p.Results.PlotSwingProjection;

% -----------------------------
% Basic checks
% -----------------------------
N = size(qr, 2);

if size(qr,1) ~= 8
    error('qr must be 8xN.');
end

if size(B,1) ~= 6 || size(B,2) ~= N
    error('B must be 6xN and must have the same number of columns as qr.');
end

if ~isempty(swingLeg)
    if numel(swingLeg) ~= N
        error('SwingLeg must be 1xN, where N = size(qr,2).');
    end
    swingLeg = reshape(swingLeg, 1, []);
end

% -----------------------------
% Allocate storage
% -----------------------------
pFeet = zeros(3,4,N);
pCoG  = zeros(3,N);
inContact = false(4,N);

legFns = {@global_leg1HTM, @global_leg2HTM, @global_leg3HTM, @global_leg4HTM};
xi_b_attach = [0.0; 1.0; 0.0; 1.0];

% -----------------------------
% Compute trajectories
% -----------------------------
for k = 1:N

    qLegs = { ...
        qr(1:2,k), ...
        qr(3:4,k), ...
        qr(5:6,k), ...
        qr(7:8,k)};

    if size(b,2) == N
        bk = b(:,k);
    else
        bk = b;
    end

    % Foot positions
    for iLeg = 1:4
        Tfoot = legFns{iLeg}(qLegs{iLeg}, 1.0, B(:,k), bk, xi_b_attach(iLeg), L, rLeg);
        pFeet(:,iLeg,k) = Tfoot(1:3,4);
    end

    % CoG position
    Tcog = global_COG(qr(:,k), 0, B(:,k), bk, 1, L, rLeg);
    pCoG(:,k) = Tcog(1:3,4);

    % Contact detection
    if ~isempty(swingLeg)
        contactNow = true(4,1);

        if swingLeg(k) >= 1 && swingLeg(k) <= 4
            contactNow(swingLeg(k)) = false;
        end

        inContact(:,k) = contactNow;

    else
        zFeet = squeeze(pFeet(3,:,k)).';

        if useRelativeGround
            zGround = min(zFeet);
            zThresh = zGround + groundPad;
        else
            zThresh = contactZThresh;
        end

        inContact(:,k) = zFeet <= zThresh;
    end
end

% -----------------------------
% Create NaN-broken foot trajectories
% -----------------------------
footX_contact = nan(4,N);
footY_contact = nan(4,N);

for iLeg = 1:4
    x = squeeze(pFeet(1,iLeg,:)).';
    y = squeeze(pFeet(2,iLeg,:)).';

    contactIdx = inContact(iLeg,:);

    footX_contact(iLeg, contactIdx) = x(contactIdx);
    footY_contact(iLeg, contactIdx) = y(contactIdx);
end

% -----------------------------
% Plot
% -----------------------------
figure;
hold on;
grid on;
box on;

% Optional support polygons
if plotSupport
    for k = 1:supportEvery:N
        idx = find(inContact(:,k));

        if numel(idx) >= 3
            XY = squeeze(pFeet(1:2,idx,k)).';

            % Robust polygon ordering
            try
                K = convhull(XY(:,1), XY(:,2));
                plot(XY(K,1), XY(K,2), '-', ...
                    'LineWidth', 0.5, ...
                    'HandleVisibility', 'off');
            catch
                % Skip degenerate polygon
            end
        end
    end
end

% Optional full foot XY projection, including swing
if plotSwingProj
    for iLeg = 1:4
        xAll = squeeze(pFeet(1,iLeg,:)).';
        yAll = squeeze(pFeet(2,iLeg,:)).';

        plot(xAll, yAll, '--', ...
            'LineWidth', 0.75, ...
            'DisplayName', sprintf('Leg %d XY projection', iLeg));
    end
end

% Foot contact trajectories with NaN gaps
for iLeg = 1:4
    plot(footX_contact(iLeg,:), footY_contact(iLeg,:), ...
        '-o', ...
        'MarkerSize',5, ...
        'LineWidth', 1.5, ...
        'DisplayName', sprintf('Leg %d contact XY', iLeg));

    % Mark start and end contact points
    validIdx = find(~isnan(footX_contact(iLeg,:)));

    if ~isempty(validIdx)
        k0 = validIdx(1);
        kf = validIdx(end);

        plot(footX_contact(iLeg,k0), footY_contact(iLeg,k0), 'o', 'MarkerSize',10, ...
            'HandleVisibility', 'off');

        plot(footX_contact(iLeg,kf), footY_contact(iLeg,kf), 'x', 'MarkerSize',10, ...
            'HandleVisibility', 'off');
    end
end

% CoG trajectory
plot(pCoG(1,:), pCoG(2,:), 'k-', ...
    'LineWidth', 2.5, ...
    'DisplayName', 'CoG XY');

plot(pCoG(1,1), pCoG(2,1), 'ko', ...
    'MarkerFaceColor', 'k', ...
    'HandleVisibility', 'off');

plot(pCoG(1,end), pCoG(2,end), 'kx', ...
    'LineWidth', 2, ...
    'HandleVisibility', 'off');

xlabel('X (m)');
ylabel('Y (m)');
title('CoG and Foot Contact Trajectories in XY Plane');
legend('Location','bestoutside');
axis equal;

% -----------------------------
% Return data
% -----------------------------
data.pFeet = pFeet;
data.pCoG = pCoG;
data.inContact = inContact;
data.footX_contact = footX_contact;
data.footY_contact = footY_contact;

end