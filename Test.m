%% Generate support functions
clear all
clc
addpath(genpath(pwd));
rmpath('Backup', 'Prep');
dt = 0.001;
FIRSTRUN = 0;

% build whole-body model for planar mc
WBMC2D = PlanarQuadruped(dt);
build2DminiCheetah(WBMC2D);

% build floating-base model for planar mc
FBMC2D = PlanarFloatingBase(dt);
build2DminiCheetahFB(FBMC2D);

if FIRSTRUN
    WBDynamics_support(WBMC2D);
    FBDynamimcs_support();
    WB_terminal_constr_support(WBMC2D);
end

%% Define a bounding gait
% One bounding gait cycle has 4 phases (1->BS,2->FL1,3->FS,4->FL2)
bounding = Gait('bounding');
currentPhase = 1; 

%% Set problem data and optimization options
problem_data.n_Phases       = 4;
problem_data.n_WBPhases     = 4;
problem_data.n_FBPhases     = 0;
problem_data.phaseSeq       = bounding.get_gaitSeq(currentPhase, problem_data.n_Phases+1);
problem_data.dt             = 0.001;
problem_data.t_horiozns     = [0.08,0.08,0.08,0.08];
problem_data.N_horizons     = floor(problem_data.t_horiozns./problem_data.dt);
problem_data.ctrl_horizon   = problem_data.N_horizons(1);
problem_data.vd             = 1.0;       % desired forward speed m/s

options.alpha            = 0.1;        % linear search update param
options.gamma            = 0.01;       % scale the expected cost reduction
options.beta_penalty     = 8;          % penalty update param
options.beta_relax       = 0.1;        % relaxation update param
options.beta_reg         = 4;          % regularization update param
options.max_DDP_iter     = 5;          % maximum DDP iterations
options.max_AL_iter      = 1;          % maximum AL iterations
options.DDP_thresh       = 0.01;       % Inner loop opt convergence threshold
options.AL_thresh        = 1e-3;       % Outer loop opt convergence threshold
options.AL_active        = 1;
options.ReB_active       = 1;
options.feedback_active  = 1;
options.Debug            = 1;

%% Define Phases info
% Construct basic unique phases
[WBPhases, FBPhases] = ConstructParentPhases(WBMC2D, FBMC2D,problem_data);

% % Preallocate HSDDP phases
Phases = repmat(BasePhase(), [problem_data.n_Phases, 1]);

% Allocate WB (whole-body) phases
for idx = 1:problem_data.n_WBPhases
    Phases(idx) = WBPhases(problem_data.phaseSeq(idx));
    Phases(idx).set_next_mode(problem_data.phaseSeq(idx+1))
    Phases(idx).set_time(problem_data.t_horiozns(idx));
end

% Allocate FB (floating-base) phases
for idx = problem_data.n_WBPhases+1:problem_data.n_Phases
    Phases(idx) = FBPhases(problem_data.phaseSeq(idx));
    Phases(idx).set_next_mode(problem_data.phaseSeq(idx+1))
    Phases(idx).set_time(problem_data.t_horiozns(idx));
end


%% Preallocate memory for trajectories of each phase
HybridTrajectory = repmat(PhaseTrajectory(),[problem_data.n_Phases, 1]);

for idx = 1:problem_data.n_Phases
    HybridTrajectory(idx) = PhaseTrajectory(Phases(idx).model.xsize,...
                                            Phases(idx).model.usize,...
                                            Phases(idx).model.ysize,...
                                            problem_data.N_horizons(idx));
end

%% Run HSDDP
% Initial condition
q0 = [0,-0.1093,-0.1542 1.0957 -2.2033 0.9742 -1.7098]';
qd0 = [0.9011 0.2756 0.7333 0.0446 0.0009 1.3219 2.7346]';
x0 = [q0;qd0];

HSDDP = HybridSystemsDDP(Phases, HybridTrajectory);

HSDDP.set_initial_condition(x0);

% AL_ReB_params = [Phases.AL_ReB_params];
% 
% HSDDP.forwardsweep(0, AL_ReB_params, options);
% 
% HSDDP.updateNominalTrajectory();
% 
% HSDDP.backwardsweep(0);

[xopt, uopt, Kopt] = HSDDP.Run(options);