%% 4-Way Intersection Plot (Teleport Style - No Animation)
% Cars are drawn at their actual X,Y positions.

global CarN CarS CarE CarW TrafficLight
global dt KK

hold off;
clf; hold on;
axis equal;
axis([-120 120 -120 120]);
xlabel('X'); ylabel('Y');
title(sprintf('Simulation time 00:%d:%02d', floor(dt*(KK-1)/60), mod(floor(dt*(KK-1)), 60)));

% === Draw Roads ===
lane_w = 3;
road_w = 2 * lane_w;

fill([-road_w/2, -road_w/2, road_w/2, road_w/2], [-120, 120, 120, -120], [0.7 0.7 0.7], 'EdgeColor', 'none');
fill([-120, 120, 120, -120], [-road_w/2, -road_w/2, road_w/2, road_w/2], [0.7 0.7 0.7], 'EdgeColor', 'none');

for offset = [-lane_w, 0, lane_w]
    plot([-120 120], [offset offset], 'w--', 'LineWidth', 1);
    plot([offset offset], [-120 120], 'w--', 'LineWidth', 1);
end

text(0, 115, 'North ↑', 'HorizontalAlignment','center', 'FontSize', 10);
text(0, -115, 'South ↓', 'HorizontalAlignment','center', 'FontSize', 10);
text(-115, 0, '← West', 'HorizontalAlignment','center', 'FontSize', 10);
text(115, 0, 'East →', 'HorizontalAlignment','center', 'FontSize', 10);

% === Draw Traffic Lights ===
r = 1.8; offset_light = 7; lw = 3;
colorMap = struct('green', [0 1 0], 'yellow', [1 1 0], 'red', [1 0 0]);

NS_color = colorMap.(TrafficLight.NS);
rectangle('Position', [-offset_light-r, -offset_light-r, 2*r, 2*r], 'Curvature', [1 1], 'FaceColor', NS_color, 'EdgeColor', 'k', 'LineWidth', lw);
rectangle('Position', [offset_light-r, offset_light-r, 2*r, 2*r], 'Curvature', [1 1], 'FaceColor', NS_color, 'EdgeColor', 'k', 'LineWidth', lw);

EW_color = colorMap.(TrafficLight.EW);
rectangle('Position', [-offset_light-r, offset_light-r, 2*r, 2*r], 'Curvature', [1 1], 'FaceColor', EW_color, 'EdgeColor', 'k', 'LineWidth', lw);
rectangle('Position', [offset_light-r, -offset_light-r, 2*r, 2*r], 'Curvature', [1 1], 'FaceColor', EW_color, 'EdgeColor', 'k', 'LineWidth', lw);

% === Draw RSU ===
global RSUObjs
if exist('RSUObjs','var') && ~isempty(RSUObjs)
    for k = 1:numel(RSUObjs)
        rsu = RSUObjs(k);
        rsu_size = 4;
        rectangle('Position', [rsu.X-rsu_size/2, rsu.Y-rsu_size/2, rsu_size, rsu_size], ...
            'Curvature', 0.2, 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'k', 'LineWidth', 1.5);
        mast_top_y = rsu.Y + rsu_size/2 + 3;
        plot([rsu.X rsu.X], [rsu.Y+rsu_size/2 mast_top_y], 'k', 'LineWidth', 1.5);
        theta = linspace(pi/3, 2*pi/3, 60);
        for rad = [3.5, 5, 6.5]
            xarc = rsu.X + rad * cos(theta);
            yarc = mast_top_y + (rad - 3) * sin(theta);
            plot(xarc, yarc, 'k', 'LineWidth', 1);
        end
        text(rsu.X, rsu.Y - rsu_size/2 - 2, sprintf('RSU %d', rsu.ID), ...
            'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
    end
end

% === Draw Cars at ACTUAL positions ===
dirColorMap = struct('N', [1 0 0], 'S', [0 1 0], 'E', [0 0 1], 'W', [1 1 0]);
turnLColor = [0.5 0.5 0.5];
turnRColor = [0.5 0 0.5];

allDirs = {'N', 'S', 'E', 'W'};
allCars = {CarN, CarS, CarE, CarW};
carW = 2; carL = 2;

for d = 1:4
    cars = allCars{d};
    dirKey = allDirs{d};
    defaultColor = dirColorMap.(dirKey);

    for j = 1:length(cars)
        car = cars(j);

        % Determine color
        if car.TurnLeft == 1
            clr = turnLColor;
        elseif car.TurnRight == 1
            clr = turnRColor;
        else
            clr = defaultColor;
        end

        % Draw at actual position based on direction
        if car.Dir == 'E' || car.Dir == 'W'
            rectangle('Position', [car.X - carL/2, car.Y - carW/2, carL, carW], ...
                'FaceColor', clr, 'EdgeColor', 'k');
        else
            rectangle('Position', [car.X - carW/2, car.Y - carL/2, carW, carL], ...
                'FaceColor', clr, 'EdgeColor', 'k');
        end
    end
end

% === Legend ===
hold on
hN = plot(nan, nan, 's', 'MarkerFaceColor', dirColorMap.N, 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'DisplayName', 'North');
hS = plot(nan, nan, 's', 'MarkerFaceColor', dirColorMap.S, 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'DisplayName', 'South');
hE = plot(nan, nan, 's', 'MarkerFaceColor', dirColorMap.E, 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'DisplayName', 'East');
hW = plot(nan, nan, 's', 'MarkerFaceColor', dirColorMap.W, 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'DisplayName', 'West');
hL = plot(nan, nan, 's', 'MarkerFaceColor', turnLColor, 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'DisplayName', 'Left turn');
hR = plot(nan, nan, 's', 'MarkerFaceColor', turnRColor, 'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'DisplayName', 'Right turn');

leg = legend([hN, hS, hE, hW, hL, hR], 'Location', 'northeastoutside');
set(leg, 'AutoUpdate', 'off');

grid on;
M(fm) = getframe(hFig);
fm = fm + 1;