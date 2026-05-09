clear

%run posture_controllers.m and trajectories.m to set parameters (posture
%controllers is not used but otherwise matlab complains about missing
%parameters in the simulation
trajectories

load_system("model.slx");

% Placeholders so the model can compile all blocks regardless of the switch
KL_params = [1, 1]; 
KNL_b = 1;
KNL_xi = 0.5;

x0_cartesian = [-1;-1;pi/4];

fprintf('preparing simulations\n');

% 
theta_pos = 0;
gamma_pos = pi/2;
delta_pos = gamma_pos+theta_pos;

err_mag = 2;
err_phase = gamma_pos;

q0_posture = [err_mag;err_phase;delta_pos];
q_d_posture = [0;0;0];


% gains
K1 = logspace(0,2,3);
K2 = logspace(0,2,3);
K3 = logspace(0,2,3);

sweep_table = combinations(K1, K2, K3);

%preparing batch
n_sims = height(sweep_table);
simInBatch(1:n_sims) = Simulink.SimulationInput('model');


for testIdx = 1:n_sims

    simIn = Simulink.SimulationInput("model");

    simIn = simIn.setModelParameter(...
        'SolverType', 'Fixed-step', ...
        'Solver', 'ode4', ...
        'FixedStep', '0.001');

    simIn = simIn.setBlockParameter('model/Manual Switch', 'sw', '0'); %simulation runs in posture control mode
    simIn = simIn.setBlockParameter('model/Manual Switch4', 'sw', '0'); %Posture control

    current_gains = [sweep_table.K1(testIdx), ...
                     sweep_table.K2(testIdx), ...
                     sweep_table.K3(testIdx)];

    simIn = simIn.setUserString(sprintf('K1_%0.1f_K2_%0.1f_K3_%0.1f', current_gains));
    simIn = simIn.setVariable('K_posture', current_gains);

    simInBatch(testIdx) = simIn;
end

fprintf('running simulations\n');
%simulation
simOutputs = parsim(simInBatch, 'TransferBaseWorkspaceVariables', 'on');
fprintf('simulations finished\n');

saveDir = fullfile('Results','posture_control');
if ~exist(saveDir, 'dir'), mkdir(saveDir); end

%output save
for sim_id = 1:length(simOutputs)
    
    out = simOutputs(sim_id);
    in = simInBatch(sim_id);
    vars = in.Variables;

    if ~isempty(out.ErrorMessage)
        fprintf('Sim %d FAILED! Skipping. Error: %s\n', sim_id, out.ErrorMessage);
        continue; % Skip to the next simulation
    end

    % Extract Data
    res = struct();
    res.time = out.tout;
    res.pose = out.yout.get('pose').Values.Data;
    %res.error = out.yout.get('e_posture').Values.Data;
    res.gains = simInBatch(sim_id).Variables(strcmp({simInBatch(sim_id).Variables.Name}, 'K_posture')).Value;
    
    % Save with a unique filename based on gains
    
    fileName = fullfile(saveDir, [simInBatch(sim_id).UserString, '.mat']);
    save(fileName, '-struct', 'res');

end
fprintf('results saved\n');