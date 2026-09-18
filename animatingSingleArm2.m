function animatingSingleArm2(l, L, r, varargin)
% Timer-based animation - FULLY interactive rotation

p = inputParser;
addParameter(p, 'FrameRate', 20);
addParameter(p, 'Loop', false);
parse(p, varargin{:});

fps = p.Results.FrameRate;
loopAnim = p.Results.Loop;

[~, columns] = size(l);
xii = linspace(0, 1, 20);
np = 21;

% Setup figure
fig = figure(1); clf;
ax = axes('Parent', fig);

% Lock axes properties
axis(ax, 'equal');
axis(ax, 'manual');
grid(ax, 'on');
xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
xlim(ax, [-0.2 0.2]); ylim(ax, [-0.2 0.2]); zlim(ax, [0 0.5]);
view(ax, 3);
daspect(ax, [1 1 1]);
ax.DataAspectRatioMode = 'manual';
camlight(ax); lighting(ax, 'gouraud');

% Enable rotation
rotate3d(ax, 'on');

% Animation state
frameIdx = 1;

% Create timer object
t = timer(...
    'ExecutionMode', 'fixedRate', ...
    'Period', 1/fps, ...
    'TimerFcn', @(~,~) updateFrame());

% Start timer
start(t);

% Nested function for frame update
    function updateFrame()
        % Generate cylinder cross-section
        [X, Y, Z] = cylinder(0.02);
        Xx = zeros(length(xii), np);
        Yy = zeros(length(xii), np);
        Zz = zeros(length(xii), np);
        
        % Transform along backbone
        for i = 1:length(xii)
            xi = xii(i);
            T = singleHTM(l(:,frameIdx), xi, L, r);
            p_circle = [X(1,:); Y(1,:); Z(1,:); ones(size(X(1,:)))];
            P = T * p_circle;
            Xx(i,:) = P(1,:);
            Yy(i,:) = P(2,:);
            Zz(i,:) = P(3,:);
        end
        
        % Delete old objects
        delete(findobj(ax, 'Type', 'surface'));
        delete(findobj(ax, 'Type', 'patch'));
        
        % Draw new frame
        surf(ax, Xx, Yy, Zz, ...
            'FaceColor', [0.3 0.6 0.3], ...
            'EdgeColor', 'none', ...
            'FaceLighting', 'gouraud');
        hold(ax, 'on');
        fill3(ax, Xx(1,:), Yy(1,:), Zz(1,:), 'g', 'EdgeColor', 'none');
        fill3(ax, Xx(end,:), Yy(end,:), Zz(end,:), 'g', 'EdgeColor', 'none');
        hold(ax, 'off');
        
        title(ax, sprintf('Frame %d/%d | l_1=%.4f m, l_2=%.4f m', ...
            frameIdx, columns, l(1,frameIdx), l(2,frameIdx)));
        
        % Advance frame
        frameIdx = frameIdx + 1;
        
        % Stop condition
        if frameIdx > columns
            if loopAnim
                frameIdx = 1;  % Loop back
            else
                stop(t);
                delete(t);
            end
        end
        
        % Check if figure closed
        if ~isvalid(fig)
            stop(t);
            delete(t);
        end
    end

end