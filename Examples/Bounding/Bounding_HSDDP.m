%% Generate support functions
clear all
clc
addpath(genpath('..\..\'));
rmpath('Backup', 'Prep');
dt = 0.001;
FIRSTRUN = 0;

% build whole-body model for planar mc
WBMC2D = PlanarQuadruped(dt);
build2DminiCheetah(WBMC2D);

% build floating-base model for planar mc
FBMC2D = PlanarFloatingBase(dt);
build2DminiCheetahFB(FBMC2D);

% If this is the first time, run support functions to generate neccessary
% functions
if FIRSTRUN
%     WBDynamics_support(WBMC2D);
    FBDynamimcs_support();
%     WB_terminal_constr_support(WBMC2D);
end

%% Define a bounding gait
% One bounding gait cycle has 4 phases (1->BS,2->FL1,3->FS,4->FL2)
bounding = Gait('bounding');
currentPhase = 1; 

%% Set problem data and optimization options
problem_data.n_Phases       = 6;       % total nuber of phases
problem_data.n_WBPhases     = 4;       % number of whole body phases
problem_data.n_FBPhases     = 2;       % number of floating-base phases
problem_data.phaseSeq       = bounding.get_gaitSeq(currentPhase, problem_data.n_Phases+1);
problem_data.dt             = dt;
problem_data.t_horizons     = [0.08,0.08,0.08,0.08,0.08,0.08];
problem_data.N_horizons     = floor(problem_data.t_horizons./problem_data.dt);
problem_data.ctrl_horizon   = problem_data.N_horizons(1);
problem_data.vd             = 1.0;     % desired forward speed m/s

options.alpha            = 0.1;        % linear search update param
options.gamma            = 0.01;       % scale the expected cost reduction
options.beta_penalty     = 8;          % penalty update param
options.beta_relax       = 0.1;        % relaxation update param
options.beta_reg         = 2;          % regularization update param
options.max_DDP_iter     = 5;          % maximum DDP iterations
options.max_AL_iter      = 4;          % maximum AL iterations
options.DDP_thresh       = 0.001;      % Inner loop opt convergence threshold
options.AL_thresh        = 1e-3;       % Outer loop opt convergence threshold
options.AL_active        = 1;          % Augmented Lagrangian active
options.ReB_active       = 1;          % Reduced barrier active
options.feedback_active  = 1;          % Smoothness active
options.Debug            = 1;          % Debug active

%% Define Phases info
% Construct basic unique phases
[WBPhases, FBPhases] = ConstructParentPhases(WBMC2D, FBMC2D,problem_data);

% % Preallocate HSDDP phases
Phases = repmat(BasePhase(), [problem_data.n_Phases, 1]);

% Allocate WB (whole-body) phases
for idx = 1:problem_data.n_WBPhases
    Phases(idx) = WBPhases(problem_data.phaseSeq(idx));
    Phases(idx).set_next_mode(problem_data.phaseSeq(idx+1))
    Phases(idx).set_time(problem_data.t_horizons(idx)); % set time of period for each phase
    Phases(idx).set_model_transition_flag(0);
end
if problem_data.n_WBPhases > 0
    Phases(problem_data.n_WBPhases).set_model_transition_flag(1);
end

% Allocate FB (floating-base) phases
for idx = problem_data.n_WBPhases+1:problem_data.n_Phases
    Phases(idx) = FBPhases(problem_data.phaseSeq(idx));
    Phases(idx).set_next_mode(problem_data.phaseSeq(idx+1))
    Phases(idx).set_time(problem_data.t_horizons(idx));
end

%% Preallocate memory for trajectories of each phase
HybridTrajectory = repmat(PhaseTrajectory(),[problem_data.n_Phases, 1]);
for idx = 1:problem_data.n_Phases
    HybridTrajectory(idx) = PhaseTrajectory(Phases(idx).model.xsize,...
                                            Phases(idx).model.usize,...
                                            Phases(idx).model.ysize,...
                                            problem_data.N_horizons(idx));
end

%% Create foldhold planner
FootPlanner = FootholdPlanner(WBMC2D, bounding, problem_data.phaseSeq(problem_data.n_WBPhases+1), ...
                      problem_data.n_FBPhases, problem_data.t_horizons(problem_data.n_WBPhases+1:end), problem_data.vd);
                            
%% Run HSDDP
% Initial condition
q0 = [0,-0.1093,-0.1542 1.0957 -2.2033 0.9742 -1.7098]';
qd0 = [0.9011 0.2756 0.7333 0.0446 0.0009 1.3219 2.7346]';
x0 = [q0;qd0];

HybridTrajectory(1).set_nom_initial_condition(x0);

heuristic_bounding_controller(WBMC2D, problem_data.phaseSeq(1:problem_data.n_WBPhases), HybridTrajectory(1:problem_data.n_WBPhases));

HSDDP = HybridSystemsDDP(Phases, HybridTrajectory);

HSDDP.set_initial_condition(x0);

HSDDP.set_FootPlanner(FootPlanner);

[xopt, uopt, Kopt] = HSDDP.Run(options);

%% Test simulator
% create simulator using planar miniCheetah model and ground at -0.404
sim = Simulator(WBMC2D);
sim.set_groundInfo(-0.404);

% Initialize delays for last phase and current phase
% Last phase delay tells simulator and controller to start at where last phase ends
% Current phase delay tells simulator to extend termination time
lastDelay = 0;
currentDelay = 0;
X = [];
for i = 1:problem_data.n_WBPhases
    % Initialize mhpc controller with planned trajectory, control and
    % feedback gain
    controller = mhpcController(xopt(i:end), uopt(i:end), Kopt(i:end));
    
    % Tell the controller the delay of last phase
    controller.InformControllerDelay(lastDelay);
    
    % Set controller to the simulator
    sim.set_Controller(controller);
    
    % Intialize simulator with scheduled phase sequence, phase horizons and
    % control horizons (to be applied)
    sim.set_horizonParams(problem_data.phaseSeq(i:end), problem_data.N_horizons(i:end),problem_data.N_horizons(i));
    
    % Recalculate the phase horizon considering the effect of last delay
    sim.recalcHorizonforDelay(lastDelay);
    
    % Run simulator
    % predidx indicates when delay = 0. This should be used as the initial
    % condition for the next MHPC planning.
    [Xphase, predidx] = sim.run(x0,currentDelay);
    
    X = [X, Xphase];
    
    x0 = Xphase(:,predix);
    
    lastDelay = currentDelay;
end


%% Visualize motion
% construct graphics and visualize data
graphicsOptions.body_active = 1;
graphicsOptions.leg_active = 1;
graphicsOptions.push_active = 1;
graphicsOptions.GRF_acitive = 1;

graphics = Graphics(get3DMCParams());
graphics.process2DData(X);
graphics.visualize( graphicsOptions);
