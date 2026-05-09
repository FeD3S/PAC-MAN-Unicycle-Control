clc
clear all
close all

run("posture_controllers.m");
run("trajectories.m");
run("tracking_controllers.m");
run("trajectories.m");

K_posture = [10; 1; 10];
x0_posture = -5;
y0_posture = -5;
theta0_posture = 0;
rho_0 = sqrt(x0_posture^2+y0_posture^2);
gamma_0 = atan2(y0_posture,x0_posture)+pi-theta0_posture;
delta_0 = gamma_0+theta0_posture;
q0_posture = [rho_0; gamma_0; delta_0];

res_global = struct();
dt = 0.001;
T_track = 25; % Duration of the whole task

% --- 1. TRAJECTORY TRACKING SETUP ---
KNL_b = 100;
KNL_xi = 0.25;
q0_dot_trajtrack = [-20; -4; 0]; % Matches start of shifted parking_trajectory

% --- 2. RUN TASK 1: TRAJECTORY TRACKING ---
load_system("full_task.slx");
simIn1 = Simulink.SimulationInput("full_task");
simIn1 = simIn1.setModelParameter('SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', num2str(dt), 'StopTime', num2str(T_track));
simIn1 = simIn1.setBlockParameter('full_task/Manual Switch', 'sw', '1');

fprintf('Starting Trajectory Tracking...\n');
out1 = sim(simIn1); % Using sim for sequential execution; use parsim if running batches

% Extract Tracking Results
t1 = out1.tout;
p1 = squeeze(out1.yout.get('pose').Values.Data)';     % Nx3
e1 = squeeze(out1.yout.get('e_b_track').Values.Data)'; % Nx3
r1 = squeeze(out1.yout.get('p_ref').Values.Data)';   % Nx3

% --- 3. PREPARE POSTURE REGULATION INITIAL CONDITIONS ---
% Get the final pose from the tracking task to ensure continuity
xf = p1(end, 1);
yf = p1(end, 2);
thf = p1(end, 3);
fprintf("final traj track pose: %d, %d, %d\n", xf, yf, thf);

% Desired Posture
q_d_posture = [0; 0; 0];
K_posture = [10,10,100];

% Calculate Initial Polar Coordinates for Sim 2 relative to q_d_posture
dx = q_d_posture(1) - xf;
dy = q_d_posture(2) - yf;
rho_0 = sqrt(dx^2 + dy^2);
gamma_0 = atan2(yf, xf) + pi - thf;
delta_0 = gamma_0 + thf;
q0_posture = [rho_0; gamma_0; delta_0];
fprintf("initial posture reg pose (for integrator): %d, %d, %d\n", rho_0, gamma_0, delta_0);

% --- 4. RUN TASK 2: POSTURE REGULATION ---
simIn2 = Simulink.SimulationInput("full_task");
simIn2 = simIn2.setModelParameter('SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', num2str(dt), 'StopTime', '10');
simIn2 = simIn2.setBlockParameter('full_task/Manual Switch', 'sw', '0');

fprintf('Starting Posture Regulation...\n');
out2 = sim(simIn2);

% Extract Posture Results
t2 = out2.tout;
p2 = squeeze(out2.yout.get('pose').Values.Data)'; % Nx3

% Calculate Posture Error
e2_x = q_d_posture(1) - p2(:,1);
e2_y = q_d_posture(2) - p2(:,2);
e2_th = mod((q_d_posture(3) - p2(:,3)) + pi, 2*pi) - pi;
e2 = [e2_x, e2_y, e2_th];

% Reference for Posture is constant
r2 = repmat(q_d_posture', size(p2, 1), 1);

% --- 5. CONCATENATE RESULTS ---
t2_shifted = t2 + t1(end);

% Store arrays in individual variables to save directly to workspace scope
time = [t1; t2_shifted];
pose = [p1; p2];
error = [e1; e2];
p_ref = [r1; r2];
switch_time = t1(end);
gains_track = [KNL_b, KNL_xi];
gains_posture = K_posture;

% --- 6. SAVE EXPLICIT VARIABLES ---
saveDir = fullfile('Results','full_tasks');
if ~exist(saveDir, 'dir'), mkdir(saveDir); end

if ~exist('folder_suffix', 'var'), folder_suffix = 'combined_sim'; end
fileName = fullfile(saveDir, [folder_suffix, '.mat']);

save(fileName, 'time', 'pose', 'error', 'p_ref', 'switch_time', 'gains_track', 'gains_posture');
fprintf('Full task simulation complete. Results saved to %s\n', fileName);