clear all
close all
clc

%% 1. LOAD TRAJECTORY DATA
load('Results/full_tasks/combined_sim.mat'); 

% Extract actual robot pose array over time: [x, y, theta]
p = pose;      
r = p_ref;     
t = time;
sw_time = switch_time;

%% 2. FIGURE SETUP
figure('Name', 'Pac-Man Animation', 'NumberTitle', 'off', ...
       'Color', 'w', 'Position', [100, 100, 1000, 700]);
hold on
grid on
axis equal

xlim([-22, 2]); 
ylim([-14, 4]);
xlabel('X Position [m]', 'FontSize', 14, 'Color', 'k');
ylabel('Y Position [m]', 'FontSize', 14, 'Color', 'k');
title('PACMAN CHALLENGE: REAL-TIME EXECUTION', 'FontSize', 16, 'FontWeight', 'bold');

% Axes aesthetics
ax = gca;
ax.FontSize = 12;
ax.LineWidth = 1.2;
ax.GridAlpha = 0.4;
ax.Box = 'on';

%% 3. PARKING BOX AND GOAL
% The parking box remains strictly centered at [0, 0].
box_width = 2.0;
box_height = 1.0;
box_x = - (box_width / 2);  
box_y = - (box_height / 2); 

% Red dashed rectangle for the parking boundary
rectangle('Position', [box_x, box_y, box_width, box_height], ...
          'EdgeColor', 'r', 'LineWidth', 2, 'LineStyle', '--');
% Star marker for the exact final goal
plot(r(end, 1), r(end, 2), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');

%% 4. DRAW STATIC OBSTACLES 
% Each row defines a strictly isolated rectangular block: [x, y, width, height].
rect_obstacles = [
    % OUTER PERIMETER (Solid frame enclosing the entire simulation)
    -22.0, -13.5,  24.0,   0.5;   % Bottom map boundary
    -22.0,   2.5,  24.0,   0.5;   % Top map boundary
    -22.0, -13.5,   0.5,  16.0;   % Left map boundary
      1.5, -13.5,   0.5,  16.0;   % Right map boundary
      
    % TOP SECTOR (Islands above the initial horizontal straight)
    -21.5,   0.5,   6.5,   1.5;   % Top-left island 1
    -15.0,   0.5,   4.0,   1.5;   % Top-left island 2
     -9.5,   0.5,   3.5,   1.0;   % Top-middle island
     -4.5,   0.5,   3.0,   1.0;   % Top-right island
     
    % LEFT SECTOR (Islands filling the space behind and below start)
    -21.5,  -6.5,   4.0,   1.5;   % Immediately below start point
    -21.5,  -9.0,   5.5,   1.5;   % Mid-left block
    -21.5, -12.0,   6.5,   2.0;   % Bottom-left block
    -19.5,  -4.0,   4.5,   1.0;   % Inside corner below start straight
    
    % CENTRAL MAZE (Fragmented "Ghost House" layout)
    -15.0,  -5.0,   2.5,   1.0;   % Top cap of the central structure 
    -15.0,  -9.0,   1.0,   4.0;   % Left wall of central structure
    -12.0,  -9.0,   1.0,   3.0;   % Right wall of central structure
    -15.0, -10.0,   4.0,   1.0;   % Bottom cap of central structure
    
    % INNER SWOOP GUIDES (Blocks catching the bottom curve of the path)
    -12.5,  -2.5,   4.5,   1.0;   % Pushes path down from the top
    -9.5,  -6.5,    1.5,   1.0;   % Inner island mid-swoop
    -15.0, -12.0,   3.5,   1.0;   % Outside guide for the bottom-left turn
    -11.5, -12.0,   3.0,   1.0;   % Bottom-center floor guide
     -5.3, -12.0,   4.3,   1.0;   % Bottom-right floor guide
     
    % ZIGZAG SUPPORT PILLARS
     -8.0, -10.0,   1.5,   8.5;   % Left vertical pillar 
     -1.5, -12.0,   1.5,   9.5;   % Right vertical pillar 
     
    % PARKING APPROACH GUIDES
     -6.5,  -1.5,   3.5,   1.0;   % Wall protecting top of final entry 
     -3.5,  -3.5,   2.0,   1.0;   % Wall guiding the bottom of the final entry

     % OTHER BLOCKS
     -0.0, -3.5,   1.5,   1.0;   % Wall at the very right
     -15.0, -2.0,   1.5,   2.5;   % Vertical Wall at the top left
     -6.0,  1.5,   1.5,   1.0;    % Wall at the very top
     -6.0,  -0.5,   1.5,   1.0;    % Wall behind the previous one

];

% Render all rectangular blocks defined in the matrix above
for i = 1:size(rect_obstacles, 1)
    obs_x = rect_obstacles(i, 1);
    obs_y = rect_obstacles(i, 2);
    obs_w = rect_obstacles(i, 3);
    obs_h = rect_obstacles(i, 4);
    rectangle('Position', [obs_x, obs_y, obs_w, obs_h], ...
              'FaceColor', [0 0.2 0.6], 'EdgeColor', 'c', 'LineWidth', 1.5);
end

% traingular blocks
tri_x = [
    -6.5, -6.5, -4.3;  
    -6.5, -6.5, -4.3;
    -6.5, -6.5, -4.3; 
    -1.5, -1.5, -4.3;  
    -1.5, -1.5, -4.3;  
];
tri_y = [
    -4.5, -6.5, -5.5;  
    -6.5, -8.5, -7.5;
    -2.5, -4.5, -3.5;
    -3.5, -5.5, -4.5;  
    -5.5, -7.5, -6.5;  
];

% Render the triangular blocks for the zigzag area
for i = 1:size(tri_x, 1)
    patch(tri_x(i,:), tri_y(i,:), [0 0.2 0.6], 'EdgeColor', 'c', 'LineWidth', 1.5);
end

% Plot trajectory tail to show path during animation. We will update its length inside the loop.
h_path = plot(p(1,1), p(1,2), 'b-', 'LineWidth', 2.0);

%% 5. INITIALIZE PAC-MAN FOR ANIMATION
% Generate base Pac-Man coordinates centered at (0,0) with theta=0 (facing right).
pac_radius = 0.4;  
mouth_angle = pi/6; 
theta_range = linspace(mouth_angle, 2*pi - mouth_angle, 50);

% Include (0,0) as the first point to form the internal mouth wedge properly
x_base = [0, pac_radius * cos(theta_range)];
y_base = [0, pac_radius * sin(theta_range)];

% Plot the initial patch object. We will update its XData and YData dynamically.
h_pacman = patch(x_base + p(1,1), y_base + p(1,2), 'y', ...
                 'EdgeColor', 'k', 'LineWidth', 1.5);

%% 6. TIME-SCALED ANIMATION LOOP
step_size = 50; 
num_frames = size(p, 1);

% Force MATLAB to completely render the background and initial pose before starting the timer.
drawnow;
fprintf('Starting time-scaled animation in 1 second...\n');
fprintf('Simulation Time: %.1f seconds.\n', t(end));
pause(1); 

% Start a high-precision real-world stopwatch
real_timer = tic;

for i = 1:step_size:num_frames
    
    % Extract current coordinates and orientation
    curr_x = p(i, 1);
    curr_y = p(i, 2);
    curr_theta = p(i, 3);
    
    % Apply 2D Rotation Matrix to the base coordinates to orient Pac-Man
    x_rot = x_base .* cos(curr_theta) - y_base .* sin(curr_theta);
    y_rot = x_base .* sin(curr_theta) + y_base .* cos(curr_theta);
    
    % Translate rotated coordinates to the current XY position
    h_pacman.XData = x_rot + curr_x;
    h_pacman.YData = y_rot + curr_y;
    
    % Update the trailing line up to the current point
    h_path.XData = p(1:i, 1);
    h_path.YData = p(1:i, 2);
    
    % Force MATLAB to paint the UI immediately for this frame
    drawnow; 
    
    % --- SCALED TIME SYNC LOGIC ---
    % Calculate the scaled real-time target for this exact frame
    target_real_time = t(i); %definito all'inizio
    
    % Check elapsed real time since the stopwatch started
    elapsed_real_time = toc(real_timer);
    
    % If we rendered faster than our target speed, pause for the 
    % difference / 2
    if elapsed_real_time < target_real_time
        pause((target_real_time - elapsed_real_time) / 2);
    end
end

% Ensure the final position is perfectly captured at the exact end of the data
h_pacman.XData = (x_base .* cos(p(end,3)) - y_base .* sin(p(end,3))) + p(end,1);
h_pacman.YData = (x_base .* sin(p(end,3)) + y_base .* cos(p(end,3))) + p(end,2);
h_path.XData = p(:, 1);
h_path.YData = p(:, 2);
drawnow;

fprintf('Animation complete.\n');
hold off