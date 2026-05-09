close all;

%% Visualize Full Task Results (Tracking + Posture)
% This script assumes 'res_global' is in the workspace or loads it from the file.

% 1. Load Data (uncomment if running standalone)
load('Results/full_tasks/combined_sim.mat'); 

% --- DATA EXTRACTION ---
t = time;
p = pose;      % Actual [x, y, theta]
r = p_ref;     % Reference [x, y, theta]
e = error;     % Error [ex, ey, etheta]
sw_time = switch_time;

% Calculate Euclidean Position Error
e_dist = sqrt(e(:,1).^2 + e(:,2).^2);
% Absolute Orientation Error
e_theta = abs(e(:,3));

% --- VISUAL STYLE SETTINGS ---
axFS = 12;      % Axis ticks
labelFS = 14;   % X/Y Labels
titleFS = 16;   % Titles
legFS = 12;     % Legend
lwMain = 2.5;   % Actual robot
lwRef = 2.0;    % Reference
swColor = [0.5 0.5 0.5]; % Gray for switch line

% Create Figure
hFig = figure('Color', 'w', 'Units', 'pixels', 'Position', [100, 100, 1300, 1000]);
tlo = tiledlayout(2,2, 'Padding', 'tight', 'TileSpacing', 'compact');

% --- TILE 1: XY path ---
ax1 = nexttile; hold on; grid on;
plot(r(:,1), r(:,2), 'r--', 'LineWidth', lwRef, 'DisplayName', 'Desired');
plot(p(:,1), p(:,2), 'b', 'LineWidth', lwMain, 'DisplayName', 'Actual');
% Mark the Switch Point
idx_sw = find(t >= sw_time, 1);
plot(p(idx_sw, 1), p(idx_sw, 2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'DisplayName', 'Switch Task');
% Mark the Final Goal
plot(r(end, 1), r(end, 2), 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'Goal');

title('XY path');
xlabel('X [m]'); ylabel('Y [m]');
axis equal; % Important for geometry
legend('Location', 'best');

% --- TILE 2: Orientation ---
ax2 = nexttile; hold on; grid on;
plot(t, r(:,3), 'r--', 'LineWidth', lwRef);
plot(t, p(:,3), 'b', 'LineWidth', lwMain);
xline(sw_time, '--', 'Color', swColor, 'LineWidth', 1.5); % Switch indicator
title('Orientation');
xlabel('Time [s]'); ylabel('\theta [rad]');
ylim([-pi-0.5, pi+0.5]);

% --- TILE 3: XY position error ---
ax3 = nexttile; hold on; grid on;
plot(t, e_dist, 'Color', [0 0.5 0], 'LineWidth', lwMain);
xline(sw_time, '--', 'Color', swColor, 'LineWidth', 1.5);
title('XY position error');
xlabel('Time [s]'); ylabel('Error [m]');

% --- TILE 4: Orientation error ---
ax4 = nexttile; hold on; grid on;
plot(t, e_theta, 'Color', [0 0.5 0], 'LineWidth', lwMain);
xline(sw_time, '--', 'Color', swColor, 'LineWidth', 1.5);
title('Orientation error');
xlabel('Time [s]'); ylabel('|Error| [rad]');

% --- GLOBAL FORMATTING ---
allAxes = findall(hFig, 'type', 'axes');
for i = 1:length(allAxes)
    ax = allAxes(i);
    ax.FontSize = axFS;
    ax.Title.FontSize = titleFS;
    ax.Title.FontWeight = 'bold';
    ax.Title.Color = [0 0 0];
    ax.XLabel.FontSize = labelFS;
    ax.XLabel.Color = [0 0 0];
    ax.YLabel.FontSize = labelFS;
    ax.YLabel.Color = [0 0 0];
    ax.XColor = [0 0 0];
    ax.YColor = [0 0 0];
    ax.LineWidth = 1.2;
    ax.GridAlpha = 0.4;
    ax.Box = 'on';
end

% Optional: Add a text annotation for the phases
annotation('textarrow',[0.25 0.2],[0.45 0.45], 'String', 'Phase 1: Tracking','FontSize',12,'FontWeight','bold');
annotation('textarrow',[0.75 0.8],[0.45 0.45], 'String', 'Phase 2: Posture','FontSize',12,'FontWeight','bold');