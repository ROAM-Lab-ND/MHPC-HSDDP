%% Generate support functions
clear all
clc
addpath(genpath('../..'));
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
    WBDynamics_support(WBMC2D);
    FBDynamimcs_support();
    WB_terminal_constr_support(WBMC2D);
end

%% Define a bounding gait
% One bounding gait cycle has 4 phases (1->BS,2->FL1,3->FS,4->FL2)
bounding = Gait('bounding');
bounding.setBasicTimings([0.08,0.08,0.08,0.08]);
currentPhase = 1; 

%% Set problem data and optimization options
problem_data.n_Phases       = 8;       % total nuber of phases
problem_data.n_WBPhases     = 8;       % number of whole body phases
problem_data.n_FBPhases     = 0;       % number of floating-base phases
problem_data.phaseSeq       = bounding.get_gaitSeq(currentPhase, problem_data.n_Phases+1);
problem_data.dt             = dt;
problem_data.t_horizons     = bounding.get_timeSeq(currentPhase, problem_data.n_Phases);
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
                            
%% Run HSDDP
% Initial condition
q0 = [0,-0.1093,-0.1542 1.0957 -2.2033 0.9742 -1.7098]';
qd0 = [0.9011 0.2756 0.7333 0.0446 0.0009 1.3219 2.7346]';
x0 = [q0;qd0];

% Initlialize mhpcController
controller = mhpcControllerNew(WBMC2D, FBMC2D, bounding, problem_data);

% create simulator using planar miniCheetah model and ground at -0.404
sim = Simulator(WBMC2D);
sim.set_groundInfo(-0.404);
% Set controller to the simulator
sim.set_Controller(controller);

% Initialize delays for last phase and current phase
% Last delay tells simulator and controller to extract some time period
% from the current phase.
lastDelay = 0;
currentDelay = 0;
x0_opt = x0;
x0_sim = x0;
X = [];

% Disturbance information
% Disturbance start at 30th time step and ends at 60th time step
disturbInfo.start = 30;
disturbInfo.end = 60;
disturbInfo.active = 0;

for i = 1:2
    controller.runHSDDP(x0_opt, options);
    
    % Tell the controller the delay of last phase
    controller.InformControllerDelay(lastDelay);       
    
    % Intialize simulator with scheduled phase sequence, phase horizons and
    % control horizons (to be applied)
    sim.set_horizonParams(problem_data.phaseSeq, problem_data.N_horizons,problem_data.N_horizons(1));
    
    % Recalculate the phase horizon considering the effect of last delay
    sim.recalcHorizonforDelay(lastDelay);
    
    % Activate disturbance at second flight
    if i == 4
        disturbInfo.active = 1;
    end
    
    % Run simulator
    % predidx indicates when delay = 0. This should be used as the initial
    % condition for the next MHPC planning.    
    [Xphase, predidx] = sim.run(x0_sim,currentDelay, disturbInfo);
    
    X = [X, Xphase];
    
    x0_opt = Xphase(:,predidx);
    
    x0_sim = Xphase(:,end);
    
    lastDelay = currentDelay;
    
    % Update problem data
    currentPhase = problem_data.phaseSeq(2);
    problem_data.phaseSeq = bounding.get_gaitSeq(currentPhase,problem_data.n_Phases+1);
    problem_data.t_horizons     = bounding.get_timeSeq(currentPhase, problem_data.n_Phases);
    problem_data.N_horizons     = floor(problem_data.t_horizons./problem_data.dt);
    controller.updateHSDDP(problem_data);
end


%% Visualize motion
% construct graphics and visualize data
graphicsOptions.body_active = 1;
graphicsOptions.leg_active = 1;
graphicsOptions.push_active = 0;
graphicsOptions.GRF_acitive = 0;

graphics = Graphics(get3DMCParams());
graphics.process2DData(X);
graphics.visualize( graphicsOptions);
