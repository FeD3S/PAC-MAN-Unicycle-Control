%% Posture Control Stats & Visualization Script - Slide Optimized
clear; clc;

% 1. Setup Paths
resultsRoot = fullfile('Results', 'posture_control');
statsRoot   = fullfile('Stats', 'posture_control');
dt = 0.001; 
q_d = [0, 0, 0]; % Desired posture [x, y, theta]

if ~exist(statsRoot, 'dir'), mkdir(statsRoot); end

% Find all .mat files
files = dir(fullfile(resultsRoot, '*.mat'));
scoreData = []; 

% --- VISUAL STYLE SETTINGS ---
axFS = 7;      % Axis ticks font
labelFS = 7;  % X/Y Labels font
titleFS = 11;  % Titles font
legFS = 9;     % Legend font
lwMain = 2.5;  % Robot path/error lines
lwRef = 1.0;   % Target indicator lines
errorColor = [0, 0.5, 0]; % Professional Green

fprintf('Found %d results. Pre-scanning for unified limits...\n', length(files));

% --- STEP 1: PRE-SCAN FOR UNIFIED LIMITS (Crucial for comparing gains on slides) ---
maxDistErr = 0.05; 
maxAngleErr = 0.05;
globalX = [-0.1, 0.1]; % Small buffer
globalY = [-0.1, 0.1];

for f = 1:length(files)
    tmp = load(fullfile(resultsRoot, files(f).name));
    p_tmp = squeeze(tmp.pose)';
    % Check XY bounds
    globalX = [min([globalX, p_tmp(:,1)']), max([globalX, p_tmp(:,1)'])];
    globalY = [min([globalY, p_tmp(:,2)']), max([globalY, p_tmp(:,2)'])];
    % Check errors
    ex = q_d(1) - p_tmp(:,1); ey = q_d(2) - p_tmp(:,2);
    dist_tmp = sqrt(ex.^2 + ey.^2);
    eth_w = mod((q_d(3) - p_tmp(:,3)) + pi, 2*pi) - pi;
    maxDistErr = max(maxDistErr, max(dist_tmp));
    maxAngleErr = max(maxAngleErr, max(abs(eth_w)));
end

% Unified Scales
distLim  = [0, maxDistErr * 1.1];
angleLim = [0, maxAngleErr * 1.1];
xLim = globalX + [-0.5, 0.5]; % Add margin
yLim = globalY + [-0.5, 0.5];

% --- STEP 2: MAIN PROCESSING LOOP ---
for f = 1:length(files)
    filename = files(f).name;
    data = load(fullfile(resultsRoot, filename));
    
    p = squeeze(data.pose)'; 
    time = data.time;
    gains = data.gains; 
    
    % 3. Calculate Errors & Scores
    ex = q_d(1) - p(:,1);
    ey = q_d(2) - p(:,2);
    eth_wrapped = mod((q_d(3) - p(:,3)) + pi, 2*pi) - pi;
    xy_err = sqrt(ex.^2 + ey.^2);
    
    score_pos = sum(time .* xy_err) * dt;
    score_theta = sum(time .* abs(eth_wrapped)) * dt;
    scoreData = [scoreData; {gains(1), gains(2), gains(3), score_pos, score_theta}];
    
    % 4. Create Folders & Save Numerical Data
    gainStr = sprintf('K1_%0.1f_K2_%0.1f_K3_%0.1f', gains(1), gains(2), gains(3));
    indivStatsFolder = fullfile(statsRoot, gainStr);
    if ~exist(indivStatsFolder, 'dir'), mkdir(indivStatsFolder); end
    
    unicycle_xy = p(:, 1:2); target_xy = [0, 0];
    save(fullfile(indivStatsFolder, 'xy_poses.mat'), 'unicycle_xy', 'target_xy');
    unicycle_theta = p(:, 3); target_theta = 0;
    save(fullfile(indivStatsFolder, 'theta_poses.mat'), 'unicycle_theta', 'target_theta');

    % 5. Generate Slide-Optimized Plot
    hFig = figure('Visible', 'off');
    hFig.Color = 'w'; 
    hFig.Units = 'pixels';
    hFig.Position = [100, 100, 1200, 900]; 
    
    tlo = tiledlayout(2,2, 'Padding', 'tight', 'TileSpacing', 'compact');
    
    % --- Tile 1: XY path ---
    ax1 = nexttile; hold on;
    plot(p(:,1), p(:,2), 'b', 'LineWidth', lwMain); 
    plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); 
    title('XY path');
    xlabel('X [m]'); ylabel('Y [m]');
    xlim(xLim); ylim(yLim); axis equal; grid on;
    
    % --- Tile 2: Orientation ---
    ax2 = nexttile; hold on;
    plot(time, p(:,3), 'b', 'LineWidth', lwMain); 
    yline(0, 'r--', 'LineWidth', lwRef); 
    title('Orientation');
    xlabel('Time [s]'); ylabel('\theta [rad]');
    ylim([-pi-0.2, pi+0.2]); grid on;
    
    % --- Tile 3: XY position error ---
    ax3 = nexttile; hold on;
    plot(time, xy_err, 'Color', errorColor, 'LineWidth', lwMain); 
    title('XY position error');
    xlabel('Time [s]'); ylabel('Error [m]');
    ylim(distLim); grid on;
    
    % --- Tile 4: Orientation error ---
    ax4 = nexttile; hold on;
    plot(time, abs(eth_wrapped), 'Color', errorColor, 'LineWidth', lwMain); 
    title('Orientation error');
    xlabel('Time [s]'); ylabel('|Error| [rad]');
    ylim(angleLim); grid on;
    
    % --- GLOBAL FORMATTING (Pure Black [0 0 0]) ---
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
        
        ax.LineWidth = 1.2; 
        ax.Box = 'on';
        ax.Color = 'w';
        ax.XColor = [0 0 0];
        ax.YColor = [0 0 0];
        
        ax.GridColor = [0.8 0.8 0.8];
        ax.GridAlpha = 0.5;
        ax.Layer = 'top'; 
    end

    % 6. Export Image using gainStr as filename
    savePath = fullfile(indivStatsFolder, [gainStr, '.png']);
    exportgraphics(hFig, savePath, 'Resolution', 300, 'BackgroundColor', 'white');
    close(hFig);
end

% 7. Save CSV Summary
if ~isempty(scoreData)
    T = cell2table(scoreData, 'VariableNames', {'K1', 'K2', 'K3', 'xy_ITAE', 'theta_ITAE'});
    writetable(T, fullfile(statsRoot, 'scores.csv'));
end

fprintf('All posture control stats generated successfully.\n');