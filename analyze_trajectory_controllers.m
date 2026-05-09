%% Trajectory Tracking Stats & Visualization Script - Full Data & Slide Ready
clear; clc;

% 1. Setup Paths
resultsRoot = fullfile('Results', 'trajectory_tracking');
statsRoot   = fullfile('Stats', 'trajectory_tracking');
refRoot     = fullfile('Results', 'trajectory_tracking', 'trajectories'); 
dt = 0.001;
controllers = {'linear', 'nonlinear'};
trajectories = {'circle', 'ellipse', 'rectangle', 'zigzag'};

% Create base stats directory if it doesn't exist
if ~exist(statsRoot, 'dir'), mkdir(statsRoot); end

% --- VISUAL STYLE SETTINGS (Slide Optimized) ---
axFS = 7;      % Axis ticks font
labelFS = 7;  % X/Y Labels font
titleFS = 11;  % Titles font
legFS = 9;     % Legend font
lwMain = 2.5;   % Robot path/error lines
lwRef = 1.0;    % Reference path lines
errorColor = [0, 0.5, 0]; % Professional Green

for t = 1:length(trajectories)
    trajName = trajectories{t};
    
    % --- LOAD REFERENCE TRAJECTORY ---
    refPath = fullfile(refRoot, ['ref_', trajName, '.mat']);
    if ~exist(refPath, 'file')
        fprintf('Warning: Reference file not found: %s\n', refPath);
        continue;
    end
    ref = load(refPath);
    p_d = ref.ref_data;      % Reference Pose (N x 3)
    ref_time = ref.ref_time; % Reference Time (N x 1)
    p_d_theta = p_d(:, 3);
    
    % --- STEP 1: PRE-SCAN FOR UNIFIED LIMITS (To show differences on slides) ---
    maxDistErr = 0.05; 
    maxAngleErr = 0.05;
    for c = 1:length(controllers)
        inputFolder = fullfile(resultsRoot, controllers{c}, trajName);
        if ~exist(inputFolder, 'dir'), continue; end
        files = dir(fullfile(inputFolder, '*.mat'));
        for f = 1:length(files)
            tmp = load(fullfile(inputFolder, files(f).name));
            e_tmp = squeeze(tmp.error)';
            p_tmp = squeeze(tmp.pose)';
            maxDistErr = max(maxDistErr, max(sqrt(e_tmp(:,1).^2 + e_tmp(:,2).^2)));
            eth_w = mod((p_d_theta - p_tmp(:,3)) + pi, 2*pi) - pi;
            maxAngleErr = max(maxAngleErr, max(abs(eth_w)));
        end
    end
    distLim  = [0, maxDistErr * 1.1];
    angleLim = [0, maxAngleErr * 1.1];
    xLim = [min(p_d(:,1)) - 1, max(p_d(:,1)) + 1];
    yLim = [min(p_d(:,2)) - 1, max(p_d(:,2)) + 1];

    for c = 1:length(controllers)
        ctrlType = controllers{c};
        inputFolder = fullfile(resultsRoot, ctrlType, trajName);
        outputFolderBase = fullfile(statsRoot, ctrlType, trajName);
        
        if ~exist(inputFolder, 'dir'), continue; end
        files = dir(fullfile(inputFolder, '*.mat'));
        scoreData = []; % Reset scores for this trajectory/controller
        
        for f = 1:length(files)
            % --- 2. LOAD SIMULATION DATA ---
            filename = files(f).name;
            data = load(fullfile(inputFolder, filename));
            
            % Reshape Data (3 x 1 x N -> N x 3)
            e = squeeze(data.error)'; 
            p = squeeze(data.pose)';
            time = data.time;


            % --- 3. CALCULATE ERRORS & SCORES ---
            e_dist = sqrt(e(:,1).^2 + e(:,2).^2);
            eth_wrapped = mod((p_d_theta - p(:,3)) + pi, 2*pi) - pi;
            
            % Calculate Numerical Scores (ITAE)
            xy_score = sum(time .* e_dist) * dt;
            angle_score = sum(time .* abs(eth_wrapped)) * dt;
            
            % Extract gains for CSV
            scoreData = [scoreData; {ctrlType, data.gains(1), data.gains(2), xy_score, angle_score}];
            
            % --- 4. CREATE FOLDERS & SAVE NUMERICAL DATA ---
            gainStr = filename(1:end-4); 
            indivStatsFolder = fullfile(outputFolderBase, gainStr);
            if ~exist(indivStatsFolder, 'dir'), mkdir(indivStatsFolder); end
            
            % Save XY Poses
            unicycle_xy = p(:, 1:2);
            traj_xy = p_d(:, 1:2);
            save(fullfile(indivStatsFolder, 'xy_poses.mat'), 'unicycle_xy', 'traj_xy');
            
            % Save Theta Poses
            unicycle_theta = p(:, 3);
            traj_theta = p_d_theta;
            save(fullfile(indivStatsFolder, 'theta_poses.mat'), 'unicycle_theta', 'traj_theta');

            % --- 5. SETUP FIGURE (Slide Optimized) ---
            hFig = figure('Visible', 'off');
            hFig.Color = 'w'; 
            hFig.Units = 'pixels';
            hFig.Position = [100, 100, 1200, 900]; % Fixed size for consistent line weight
            
            t_layout = tiledlayout(2,2, 'Padding', 'tight', 'TileSpacing', 'compact');
    
            % --- Plot 1: XY path ---
            ax1 = nexttile; hold on;
            plot(p_d(:,1), p_d(:,2), 'r--', 'LineWidth', lwRef); 
            plot(p(:,1), p(:,2), 'b', 'LineWidth', lwMain);      
            title('XY path');
            xlabel('X [m]'); ylabel('Y [m]');
            xlim(xLim); ylim(yLim);
            axis equal; grid on;
            legend('Ref', 'Robot', 'FontSize', legFS, 'Location', 'best');
    
            % --- Plot 2: Orientation ---
            ax2 = nexttile; hold on;
            plot(ref_time, p_d_theta, 'r--', 'LineWidth', lwRef); 
            plot(time, p(:,3), 'b', 'LineWidth', lwMain);        
            title('Orientation');
            xlabel('Time [s]'); ylabel('\theta [rad]');
            ylim([-pi-0.2, pi+0.2]); grid on;
    
            % --- Plot 3: XY position error ---
            ax3 = nexttile; hold on;
            plot(time, e_dist, 'Color', errorColor, 'LineWidth', lwMain); 
            title('XY position error');
            xlabel('Time [s]'); ylabel('Error [m]');
            ylim(distLim); grid on;
    
            % --- Plot 4: Orientation error ---
            ax4 = nexttile; hold on;
            plot(time, abs(eth_wrapped), 'Color', errorColor, 'LineWidth', lwMain); 
            title('Orientation error');
            xlabel('Time [s]'); ylabel('|Error| [rad]');
            ylim(angleLim); grid on;
    
            % --- GLOBAL FORMATTING (Force Pure Black & Consistent Weight) ---
            allAxes = findall(hFig, 'type', 'axes');
            for i = 1:length(allAxes)
                ax = allAxes(i);
                ax.FontSize = axFS;
                
                ax.Title.FontSize = titleFS;
                ax.Title.FontWeight = 'bold';
                ax.Title.Color = [0 0 0]; % Pure Black
                
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

            % --- 6. EXPORT PLOTS ---
            savePath = fullfile(indivStatsFolder, [gainStr, '.png']);
            exportgraphics(hFig, savePath, 'Resolution', 300, 'BackgroundColor', 'white');
            close(hFig);
        end
        
        % --- 7. SAVE SCORE CSV (One per Traj/Controller) ---
        if ~isempty(scoreData)
            T = cell2table(scoreData, 'VariableNames', {'type', 'ab', 'xi', 'xy_distance', 'angle_disp'});
            writetable(T, fullfile(outputFolderBase, 'scores.csv'));
        end
        fprintf('Processed: %s - %s\n', ctrlType, trajName);
    end
end
fprintf('Stats generated successfully: Numerical .mat files, CSV scores, and high-quality plots.\n');