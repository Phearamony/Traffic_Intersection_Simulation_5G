function plot_EW_trajectories(LightLog, CarLogE, CarLogW, stop_line, outFile, opts)
% Clean version: all trajectories black, only traffic light color at y=0.
% x-axis = time [s], y-axis = signed distance to stop line (0 = stop line)

if nargin < 5 || isempty(outFile), outFile = 'EW.png'; end
if nargin < 6, opts = struct; end

% Options
draw_guides = getfielddef(opts,'draw_guides',false); % disable guide lines
v_free      = getfielddef(opts,'v_free',12);
light_width = getfielddef(opts,'light_width',5);

% Time range
tMax = 0;
if ~isempty(LightLog), tMax = max(tMax, max([LightLog.Time])); end
if ~isempty(CarLogE),  tMax = max(tMax, max([CarLogE.Time]));  end
if ~isempty(CarLogW),  tMax = max(tMax, max([CarLogW.Time]));  end
if tMax <= 0, warning('No data to plot.'); return; end

% === Figure ===
hFig = figure('Name','EW Trajectories @ y=0','Position',[120 100 1000 560]);

% ===== (a) EAST =====
hAx1 = subplot(2,1,1); hold(hAx1,'on'); grid(hAx1,'on');
title(hAx1,'(a) Trajectories of cars from East');
xlabel(hAx1,'Time [s]'); ylabel(hAx1,'Distance to stop line [m]');
xlim(hAx1,[0 tMax]);

% traffic light band
paint_light_strip_y0(hAx1, LightLog, 'EW', tMax, light_width);

% Optional guide lines
if draw_guides
    y0s = 5:5:60;
    for k = 1:numel(y0s)
        t1 = y0s(k)/max(v_free,0.1);
        plot(hAx1,[0 min(t1,tMax)],[y0s(k) max(0,y0s(k)-v_free*min(t1,tMax))], ...
             'Color',[0.6 0.6 0.6],'LineWidth',0.75);
    end
end

% Plot all E trajectories (black)
plot_traj_black(hAx1, CarLogE, @(X) max(0, (-stop_line) - X));

ylim(hAx1,[0 auto_ylim(hAx1)]);

% ===== (b) WEST =====
hAx2 = subplot(2,1,2); hold(hAx2,'on'); grid(hAx2,'on');
title(hAx2,'(b) Trajectories of cars from West');
xlabel(hAx2,'Time [s]'); ylabel(hAx2,'Distance to stop line [m]');
xlim(hAx2,[0 tMax]);

paint_light_strip_y0(hAx2, LightLog, 'EW', tMax, light_width);

if draw_guides
    y0s = -5:-5:-60;
    for k = 1:numel(y0s)
        t1 = abs(y0s(k))/max(v_free,0.1);
        plot(hAx2,[0 min(t1,tMax)],[y0s(k) min(0,y0s(k)+v_free*min(t1,tMax))], ...
             'Color',[0.6 0.6 0.6],'LineWidth',0.75);
    end
end

plot_traj_black(hAx2, CarLogW, @(X) -max(0, X - stop_line));

ylim(hAx2,[-auto_ylim(hAx2) 0]);

exportgraphics(hFig, outFile, 'Resolution',300);
fprintf('Saved clean figure to %s\n', outFile);
end

% ======= HELPERS =======
function paint_light_strip_y0(ax, LLog, whichField, tmax, light_width)
if isempty(LLog), return; end
times  = [LLog.Time];
phases = lower(string({LLog.(whichField)}));
i0 = 1;
for k = 2:numel(times)
    if phases(k) ~= phases(k-1)
        drawSeg(times(i0), times(k), phases(k-1), ax, light_width);
        i0 = k;
    end
end
drawSeg(times(i0), tmax, phases(end), ax, light_width);
plot(ax,[0 tmax],[0 0],'k-','LineWidth',1);
end

function drawSeg(t1,t2,phase,ax,light_width)
if t2 <= t1, return; end
switch phase
    case 'green',  c = [0 .7 0];
    case 'yellow', c = [.95 .75 0];
    otherwise,     c = [.8 0 0];
end
dy = light_width/2;
patch(ax,[t1 t2 t2 t1],[-dy -dy +dy +dy],c,'EdgeColor','none','FaceAlpha',0.95);
end

function plot_traj_black(ax, CLog, distFun)
if isempty(CLog), return; end
ids = unique([CLog.ID]);
for ii = 1:numel(ids)
    id = ids(ii);
    sel = CLog([CLog.ID]==id);
    [t,idx] = sort([sel.Time]); sel = sel(idx);
    y = distFun([sel.X]);
    plot(ax,t,y,'k','LineWidth',0.9); % pure black lines only
end
end

function ym = auto_ylim(ax)
L = findobj(ax,'Type','line');
if isempty(L), ym = 50; return; end
ys = [];
for j=1:numel(L)
    ys = [ys; L(j).YData(:)];
end
ym = max(10, prctile(abs(ys),98)*1.1);
end

function v = getfielddef(s,f,d)
if isstruct(s) && isfield(s,f), v = s.(f); else, v = d; end
end

