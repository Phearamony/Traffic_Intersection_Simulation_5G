function plot_NS_trajectories(LightLog, CarLogN, CarLogS, stop_line, outFile, opts)
% PLOT_NS_TRAJECTORIES Clean version: all trajectories black, only traffic light color at y=0.
%   x-axis = time [s], y-axis = signed distance to stop line (0 = stop line)
%
%   INPUTS:
%       LightLog  - Structure array with fields: Time, NS (traffic light phases)
%       CarLogN   - Structure array with fields: ID, Time, Y (cars from North)
%       CarLogS   - Structure array with fields: ID, Time, Y (cars from South)
%       stop_line - Scalar, Y position of the stop line
%       outFile   - (Optional) Output filename (default: 'NS.png')
%       opts      - (Optional) Structure with options:
%                   .draw_guides - Enable guide lines (default: false)
%                   .v_free      - Free flow velocity (default: 12)
%                   .light_width - Width of light strip (default: 5)

    if nargin < 5 || isempty(outFile)
        outFile = 'NS.png';
    end
    if nargin < 6
        opts = struct;
    end

    % Options
    draw_guides = getfielddef(opts, 'draw_guides', false);
    v_free      = getfielddef(opts, 'v_free', 12);
    light_width = getfielddef(opts, 'light_width', 5);

    % Time range
    tMax = 0;
    if ~isempty(LightLog)
        tMax = max(tMax, max([LightLog.Time]));
    end
    if ~isempty(CarLogN)
        tMax = max(tMax, max([CarLogN.Time]));
    end
    if ~isempty(CarLogS)
        tMax = max(tMax, max([CarLogS.Time]));
    end
    if tMax <= 0
        warning('No data to plot.');
        return;
    end

    % === Figure ===
    hFig = figure('Name', 'NS Trajectories @ x=0', 'Position', [120 100 1000 560]);

    % ===== (a) North =====
    % Cars from North travel in -Y direction (approaching stop line from above)
    hAx1 = subplot(2, 1, 1);
    hold(hAx1, 'on');
    grid(hAx1, 'on');
    title(hAx1, '(a) Trajectories of cars from North');
    xlabel(hAx1, 'Time [s]');
    ylabel(hAx1, 'Distance to stop line [m]');
    xlim(hAx1, [0 tMax]);

    % Traffic light band
    paint_light_strip_y0(hAx1, LightLog, 'NS', tMax, light_width);

    % Optional guide lines
    if draw_guides
        y0s = 5:5:60;
        for k = 1:numel(y0s)
            t1 = y0s(k) / max(v_free, 0.1);
            plot(hAx1, [0 min(t1, tMax)], [y0s(k) max(0, y0s(k) - v_free * min(t1, tMax))], ...
                'Color', [0.6 0.6 0.6], 'LineWidth', 0.75);
        end
    end

    % Plot all N trajectories (black)
    % Cars from North: Y > stop_line, approaching from positive Y
    % Distance = Y - stop_line (positive when above stop line)
    plot_traj_black(hAx1, CarLogN, @(Y) max(0, Y - stop_line));
    ylim(hAx1, [0 auto_ylim(hAx1)]);

    % ===== (b) South =====
    % Cars from South travel in +Y direction (approaching stop line from below)
    hAx2 = subplot(2, 1, 2);
    hold(hAx2, 'on');
    grid(hAx2, 'on');
    title(hAx2, '(b) Trajectories of cars from South');
    xlabel(hAx2, 'Time [s]');
    ylabel(hAx2, 'Distance to stop line [m]');
    xlim(hAx2, [0 tMax]);

    paint_light_strip_y0(hAx2, LightLog, 'NS', tMax, light_width);

    if draw_guides
        y0s = -5:-5:-60;
        for k = 1:numel(y0s)
            t1 = abs(y0s(k)) / max(v_free, 0.1);
            plot(hAx2, [0 min(t1, tMax)], [y0s(k) min(0, y0s(k) + v_free * min(t1, tMax))], ...
                'Color', [0.6 0.6 0.6], 'LineWidth', 0.75);
        end
    end

    % Plot all S trajectories (black)
    % Cars from South: Y < stop_line, approaching from negative Y
    % Distance = -(stop_line - Y) (negative when below stop line)
    plot_traj_black(hAx2, CarLogS, @(Y) -max(0, stop_line - Y));
    ylim(hAx2, [-auto_ylim(hAx2) 0]);

    % Export figure
    exportgraphics(hFig, outFile, 'Resolution', 300);
    fprintf('Saved clean figure to %s\n', outFile);

end

%% ======= HELPER FUNCTIONS =======

function paint_light_strip_y0(ax, LLog, whichField, tmax, light_width)
% Paint traffic light phases as colored strips at y=0
    if isempty(LLog)
        return;
    end
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
    plot(ax, [0 tmax], [0 0], 'k-', 'LineWidth', 1);
end

function drawSeg(t1, t2, phase, ax, light_width)
% Draw a single segment of the traffic light strip
    if t2 <= t1
        return;
    end
    switch phase
        case 'green'
            c = [0 0.7 0];
        case 'yellow'
            c = [0.95 0.75 0];
        otherwise
            c = [0.8 0 0];
    end
    dy = light_width / 2;
    patch(ax, [t1 t2 t2 t1], [-dy -dy +dy +dy], c, ...
        'EdgeColor', 'none', 'FaceAlpha', 0.95);
end

function plot_traj_black(ax, CLog, distFun)
% Plot all trajectories in black using Y coordinate
    if isempty(CLog)
        return;
    end
    ids = unique([CLog.ID]);
    for ii = 1:numel(ids)
        id = ids(ii);
        sel = CLog([CLog.ID] == id);
        [t, idx] = sort([sel.Time]);
        sel = sel(idx);
        y = distFun([sel.Y]);
        plot(ax, t, y, 'k', 'LineWidth', 0.9);
    end
end

function ym = auto_ylim(ax)
% Automatically determine y-axis limit based on data
    L = findobj(ax, 'Type', 'line');
    if isempty(L)
        ym = 50;
        return;
    end
    ys = [];
    for j = 1:numel(L)
        ys = [ys; L(j).YData(:)]; %#ok<AGROW>
    end
    ym = max(10, prctile(abs(ys), 98) * 1.1);
end

function v = getfielddef(s, f, d)
% Get field from struct with default value
    if isstruct(s) && isfield(s, f)
        v = s.(f);
    else
        v = d;
    end
end