%% Trajectory Reference Generator
clear; bdclose('all');

trajectories;
tracking_controllers;
posture_controllers;


load_system("model.slx");

% Define the 4 trajectories and their switch states (matching your previous setup)
traj_names = {'rectangle', 'zigzag', 'circle', 'ellipse'};
traj_switches = {{'1','1'}, {'1','0'}, {'0','1'}, {'0','0'}}; 

% Path to the Master "Trajectory vs Posture" switch (Assume 1 is Trajectory)
master_switch = 'model/Manual Switch'; 

if ~exist('Results/trajectory_tracking/trajectories', 'dir'), mkdir('Results/trajectory_tracking/trajectories'); end

for i = 1:length(traj_names)
    fprintf('Generating %s trajectory...\n', traj_names{i});
    
    simIn = Simulink.SimulationInput("model");
    
    % Force the model into Trajectory Mode
    simIn = simIn.setBlockParameter(master_switch, 'sw', '1');
    
    % Set the specific trajectory switches (5, 6, and 7)
    if i <= 2 % Rectangle or Zigzag
        simIn = simIn.setBlockParameter('model/Manual Switch5', 'sw', traj_switches{i}{2});
    else      % Circle or Ellipse
        simIn = simIn.setBlockParameter('model/Manual Switch6', 'sw', traj_switches{i}{2});
    end
    simIn = simIn.setBlockParameter('model/Manual Switch7', 'sw', traj_switches{i}{1});

    % Run the simulation (we only need one run per type)
    out = sim(simIn);
    
    % Extract the reference pose (p_d)
    % Note: Replace 'p_ref' with whatever you named your Outport
    ref_data = out.yout.get('p_ref').Values.Data;
    ref_time = out.tout;
    
    % Save to the References folder
    saveName = fullfile('Results/trajectory_tracking/trajectories', ['ref_', traj_names{i}, '.mat']);
    save(saveName, 'ref_data', 'ref_time');
end

fprintf('All reference trajectories saved in ./References folder.\n');