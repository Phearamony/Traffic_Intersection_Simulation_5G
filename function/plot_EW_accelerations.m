function plot_EW_accelerations(LightLog, CarLogE, CarLogW, ~, outFile, opts)
% Acceleration plot: all trajectories black, traffic light color strip.
% x-axis = time [s], y-axis = acceleration [m/s^2]
% East subplot: light strip at top (y=0 line area)
% West subplot: light strip at bottom
% Note: 4th argument (stop_line) is ignored, kept for compatibility
if nargin < 5 || isempty(outFile), outFile = 'EW_accel.png'; end
if nargin < 6, opts = struct; end

% Options
light_width = getfielddef(opts, 'light_width', 1);  % thinner for accel plot

% Time range
tMax = 0;
if ~isempty(LightLog), tMax = max(tMax, max([LightLog.Time])); end
if ~isempty(CarLogE),  tMax = max(tMax, max([CarLogE.Time]));  end
if ~isempty(CarLogW),  tMax = max(tMax, max([CarLogW.Time]));  end
if tMax <= 0, warning('No data to plot.'); return; end

% === Figure ===
hFig = figure('Name','EW Accelerations','Position',[120 100 1000 560]);

% ===== (a) EAST =====
hAx1 = subplot(2,1,1); hold(hAx1,'on'); grid(hAx1,'on');
title(hAx1,'(a) Acceleration of cars from East');
xlabel(hAx1,'Time [s]'); ylabel(hAx1,'Acceleration [m/s^2]');
xlim(hAx1,[0 tMax]);

% Plot all E trajectories (black) - acceleration
plot_accel_black(hAx1, CarLogE);

% Get y-limits first, then paint light strip at top
yl = auto_ylim_accel(hAx1);
ylim(hAx1, [-yl yl]);

% Traffic light band at top (near positive y limit)
paint_light_strip_at_y(hAx1, LightLog, 'EW', tMax, light_width, yl - light_width/2);

% ===== (b) WEST =====
hAx2 = subplot(2,1,2); hold(hAx2,'on'); grid(hAx2,'on');
title(hAx2,'(b) Acceleration of cars from West');
xlabel(hAx2,'Time [s]'); ylabel(hAx2,'Acceleration [m/s^2]');
xlim(hAx2,[0 tMax]);

% Plot all W trajectories (black) - acceleration (no flipping!)
plot_accel_black(hAx2, CarLogW);

% Get y-limits
yl2 = auto_ylim_accel(hAx2);
ylim(hAx2, [-yl2 yl2]);

% Traffic light band at BOTTOM (near negative y limit)
paint_light_strip_at_y(hAx2, LightLog, 'EW', tMax, light_width, -yl2 + light_width/2);

% Export
saveas(hFig, outFile);
fprintf('Saved acceleration figure to %s\n', outFile);
end

% ======= HELPERS =======

function paint_light_strip_at_y(ax, LLog, whichField, tmax, light_width, y_center)
% Paint traffic light strip centered at y_center
if isempty(LLog), return; end
times  = [LLog.Time];
phases = lower(string({LLog.(whichField)}));
i0 = 1;
for k = 2:numel(times)
    if phases(k) ~= phases(k-1)
        drawSeg(times(i0), times(k), phases(k-1), ax, light_width, y_center);
        i0 = k;
    end
end
drawSeg(times(i0), tmax, phases(end), ax, light_width, y_center);
% Draw center line through the strip
plot(ax, [0 tmax], [y_center y_center], 'k-', 'LineWidth', 0.5);
end

function drawSeg(t1, t2, phase, ax, light_width, y_center)
if t2 <= t1, return; end
switch phase
    case 'green',  c = [0 .7 0];
    case 'yellow', c = [.95 .75 0];
    otherwise,     c = [.8 0 0];
end
dy = light_width / 2;
patch(ax, [t1 t2 t2 t1], [y_center-dy y_center-dy y_center+dy y_center+dy], ...
      c, 'EdgeColor', 'none', 'FaceAlpha', 0.95);
end

function plot_accel_black(ax, CLog)
% Plot acceleration for each car ID
if isempty(CLog), return; end

ids = unique([CLog.ID]);
for ii = 1:numel(ids)
    id = ids(ii);
    sel = CLog([CLog.ID] == id);
    [t, idx] = sort([sel.Time]); 
    sel = sel(idx);
    a = [sel.Ac];
    plot(ax, t, a, 'k', 'LineWidth', 0.9);
end
end

function ym = auto_ylim_accel(ax)
% Automatically determine y-limits for acceleration plot
L = findobj(ax, 'Type', 'line');
if isempty(L)
    ym = 5;  % default acceleration range
    return;
end
ys = [];
for j = 1:numel(L)
    ys = [ys; L(j).YData(:)];
end
ys = ys(isfinite(ys));  % remove NaN/Inf
if isempty(ys)
    ym = 5;
else
    ym = max(3, prctile(abs(ys), 98) * 1.15);  % add some margin
end
end

function v = getfielddef(s, f, d)
if isstruct(s) && isfield(s, f)
    v = s.(f);
else
    v = d;
end
end