clear

%run posture_controllers.m and trajectories.m to set parameters (posture
%controllers is not used but otherwise matlab complains about missing
%parameters in the simulation
posture_controllers
trajectories

load_system("model.slx");

% Placeholders so the model can compile all blocks regardless of the switch
KL_params = [1, 1]; 
KNL_b = 1;
KNL_xi = 0.5;
%the important values will be replaced by the actual gains

fprintf('preparing simulations\n');

%linear controller
%KL_a = logspace(0,3,10);
%KL_xi = linspace(0,1,12);
%KL_xi = KL_xi(2:end-1);

KL_a_ = logspace(0,2,3);
KL_xi_ = linspace(0,1,5);
KL_xi_ = KL_xi_(2:end-1);

%non linear controller
%KNL_b = logspace(0,3,10);
%KNL_xi = linspace(0,1,12);
%KNL_xi = KNL_xi(2:end-1);

KNL_b_ = logspace(0,2,3);
KNL_xi_ = linspace(0,1,5);
KNL_xi_ = KNL_xi_(2:end-1);

trajectories_names = {'rectangle', 'zigzag', 'circle', 'ellipse'};
trajectories_switches = {{'1','1'}, {'1','0'}, {'0','1'}, {'0','0'}};

controllers_names = {'linear', 'nonlinear'};
linear_nonlinear_controller_switch = {'1', '0'};

%preparing batch
n_sims = length(trajectories_names)*(length(KL_a_)*length(KL_xi_) + length(KNL_b_)*length(KNL_xi_));
simInBatch(1:n_sims) = Simulink.SimulationInput('model');


testIdx = 1;
for traj_id = 1:length(trajectories_names)
    %simulating trajectory trajectories_names{traj_id}
    for controller_id = 1:length(controllers_names)

        if controller_id == 1
            [A, XI] = ndgrid(KL_a_, KL_xi_);
            gains_to_test = [A(:), XI(:)];
        elseif controller_id == 2
            [NB, NXI] = ndgrid(KNL_b_, KNL_xi_);
            gains_to_test = [NB(:), NXI(:)];
        end

        for g = 1:size(gains_to_test, 1)
            simIn = Simulink.SimulationInput("model");

            simIn = simIn.setModelParameter(...
                'SolverType', 'Fixed-step', ...
                'Solver', 'ode4', ...
                'FixedStep', '0.001');

            simIn = simIn.setBlockParameter('model/Manual Switch', 'sw', '1'); %simulation runs in trajectory tracking mode
            simIn = simIn.setBlockParameter('model/Manual Switch1', 'sw', num2str(linear_nonlinear_controller_switch{controller_id})); %Linear/non linear switch
            simIn = simIn.setBlockParameter('model/Manual Switch2', 'sw', '1'); %State/output error switch

            %trajectory switch setting
            if traj_id == 1 || traj_id == 2
                simIn = simIn.setBlockParameter('model/Manual Switch5', 'sw', trajectories_switches{traj_id}{2});
            elseif traj_id == 3 || traj_id == 4
                simIn = simIn.setBlockParameter('model/Manual Switch6', 'sw', trajectories_switches{traj_id}{2});
            end
            simIn = simIn.setBlockParameter('model/Manual Switch7', 'sw', trajectories_switches{traj_id}{1});

            if controller_id == 1
                simIn = simIn.setVariable('KL_params', gains_to_test(g, :)); %KL_params = [a, xi]
            elseif controller_id == 2
                simIn = simIn.setVariable('KNL_b', gains_to_test(g, 1)); %b
                simIn = simIn.setVariable('KNL_xi', gains_to_test(g, 2)); %xi

            end

            simIn = simIn.setUserString(trajectories_names{traj_id});

            simInBatch(testIdx) = simIn;
            testIdx = testIdx +1;
        end
    end
end

fprintf('running simulations\n');
%simulation
simOutputs = parsim(simInBatch, 'TransferBaseWorkspaceVariables', 'on');
fprintf('simulations finished\n');
%output save
for sim_id = 1:length(simOutputs)
    
    out = simOutputs(sim_id);
    in = simInBatch(sim_id);
    vars = in.Variables;

    traj_name = in.UserString;


    if ~isempty(out.ErrorMessage)
        fprintf('Sim %d FAILED! Skipping. Error: %s\n', sim_id, out.ErrorMessage);
        continue; % Skip to the next simulation
    end


    ctrl_switch = in.getBlockParameter('model/Manual Switch1', 'sw'); 
    if strcmp(ctrl_switch, '1')
        ctrl_name = 'linear';
        idx = strcmp({vars.Name}, 'KL_params');
        gains_used = vars(idx).Value;
        folder_suffix = sprintf('a_%0.1f_xi_%0.2f', gains_used(1), gains_used(2));
    else
        ctrl_name = 'nonlinear';
        idx_b = strcmp({vars.Name}, 'KNL_b');
        idx_xi = strcmp({vars.Name}, 'KNL_xi');
        gains_used = [vars(idx_b).Value, vars(idx_xi).Value];
        folder_suffix = sprintf('b_%0.1f_xi_%0.2f', gains_used(1), gains_used(2));
    end

    res = struct();
    res.time = out.tout;
    res.pose = out.yout.get('pose').Values.Data;
    res.error = out.yout.get('e_b_track').Values.Data;
    res.gains = gains_used;
    res.trajectory = traj_name;
    
    saveDir = fullfile('Results','trajectory_tracking', ctrl_name, traj_name);
    if ~exist(saveDir, 'dir'), mkdir(saveDir); end

    fileName = fullfile(saveDir, [folder_suffix, '.mat']);
    save(fileName, '-struct', 'res');
end
fprintf('results saved\n');