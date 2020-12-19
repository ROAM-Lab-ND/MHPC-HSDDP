function [WBPhases, FBPhases] = ConstructParentPhasesJumping(WBMC, FBMC, params)
% This function creates four base phases for each model
% 1-> BS 2->FL1 3->FS 4->FL2
JumpConstraint = constraint(WBMC, FBMC);
JumpCost       = cost(WBMC, FBMC);

% Initialize WBPhases and FBPhases
for mode = 1:4
    WBPhases(mode) = BasePhase(WBMC, mode, 'full');
    FBPhases(mode) = BasePhase(FBMC, mode, 'simple');
end

%% Set constraints
% The terminal constraint and ineq constraint are inactive by default
% WB constraints
WBPhases(1).set_ineq_constraint(@JumpConstraint.WB_jumping_ineq_constr);
WBPhases(1).set_constraint_active('ineq'); 

WBPhases(2).set_terminal_constraint(@JumpConstraint.WB_bounding_tm_constr);
WBPhases(2).set_constraint_active('terminal');
WBPhases(2).set_ineq_constraint(@JumpConstraint.WB_jumping_ineq_constr);
WBPhases(2).set_constraint_active('ineq');

WBPhases(3).set_ineq_constraint(@JumpConstraint.WB_jumping_ineq_constr);
WBPhases(3).set_constraint_active('ineq');

WBPhases(4).set_terminal_constraint(@JumpConstraint.WB_bounding_tm_constr);
WBPhases(4).set_constraint_active('terminal');
WBPhases(4).set_ineq_constraint(@JumpConstraint.WB_jumping_ineq_constr);
WBPhases(4).set_constraint_active('ineq');

% FB constraints

%% set quadratic cost weightings
% Preallocate memory to accelerate computation
Qwb     = zeros(WBMC.xsize, WBMC.xsize, 4);
Rwb     = zeros(WBMC.usize, WBMC.usize, 4);
Swb     = zeros(WBMC.ysize, WBMC.ysize, 4);
Qwbf    = zeros(WBMC.xsize, WBMC.xsize, 4);

Qfb     = zeros(FBMC.xsize, FBMC.xsize, 4);
Rfb     = zeros(FBMC.usize, FBMC.usize, 4);
Sfb     = zeros(FBMC.ysize, FBMC.ysize, 4);
Qfbf    = zeros(FBMC.xsize, FBMC.xsize, 4);

% weightings for WB (whole-body) model
% BS
Qwb(:,:,1)      = 0.01*diag([0,10,5,2,2,2,2, 8,1,.01,5,5,5,5]);
Rwb(:,:,1)      = diag([5,5,1,1]);
Swb(3:4,3:4,1)  = 0.3*eye(2);
Qwbf(:,:,1)     = 100*diag([10,20,8,4,4,4,4, 5,2,0.01,3,3,0.01,0.01]);  

% FL1
Qwb(:,:,2)      = 0.01*diag([0,10,5,2,2,2,2,8,1,.01,2,2,2,2]); 
Rwb(:,:,2)      = eye(WBMC.usize);
Swb(:,:,2)      = zeros(WBMC.ysize);
Qwbf(:,:,2)     = 100*diag([10,20,8,4,4,4,4,5,1,0.01,2,2,2,2]); 

% FS
Qwb(:,:,3)      = 0.01*diag([0,10,5,2,2,2,2,8,1,.01,2,2,2,2]);
Rwb(:,:,3)      = diag([1,1,5,5]);
Swb(1:2,1:2,3)  = 0.15*eye(2);
Qwbf(:,:,3)     = 100*diag([10,20,8,4,4,4,4, 10,2,0.01,0.01,0.01,3,3]);

% FL2
Qwb(:,:,4)      = 0.01*diag([0,10,5,2,2,2,2, 8,1,.01,2,2,2,2]);
Rwb(:,:,4)      = eye(WBMC.usize);
Swb(:,:,4)      = zeros(WBMC.ysize);
Qwbf(:,:,4)     = 100*diag([10,20,8,4,4,4,4, 10,2,0.01,2,2,2,2]);

% weightings for FB (floating-base) model
% BS
Qfb(:,:,1)  = 0.01*diag([0,10,5,8,5,0.01]);
Rfb(3:4,3:4,1)  = 0.2*eye(2);
Sfb(:,:,1)  = zeros(FBMC.ysize);
Qfbf(:,:,1) = 100*diag([10,20,8,10,8,0.01]); 

% FL1
Qfb(:,:,2)  = 0.01*diag([0,10,5,8,2,0.01]);
Rfb(:,:,2)  = zeros(FBMC.usize);  
Sfb(:,:,2)  = zeros(FBMC.ysize);
Qfbf(:,:,2) = 100*diag([10,20,8,10,1,0.01]); 

% FS
Qfb(:,:,3)  = 0.01*diag([0,10,5,8,5,0.01]);
Rfb(1:2,1:2,3)  = 0.1*eye(2);
Sfb(:,:,3)  = zeros(FBMC.ysize);
Qfbf(:,:,3) = 100*diag([10,20,8,10,8,0.01]);  

% FL2
Qfb(:,:,4)  = 0.01*diag([0,10,5,8,2,0.01]);
Rfb(:,:,4)  = zeros(FBMC.usize);
Sfb(:,:,4)  = zeros(FBMC.ysize);
Qfbf(:,:,4) = 100*diag([10,20,8,10,1,0.01]); 

for mode = 1:4
    WBPhases(mode).set_running_cost(@JumpCost.running_cost_Info);
    WBPhases(mode).set_terminal_cost(@JumpCost.jump_terminal_cost_Info);
    WBPhases(mode).set_weightings(Qwb(:,:,mode),Rwb(:,:,mode),Swb(:,:,mode),Qwbf(:,:,mode));
    
    FBPhases(mode).set_running_cost(@JumpCost.running_cost_Info);
    FBPhases(mode).set_terminal_cost(@JumpCost.terminal_cost_Info);
    FBPhases(mode).set_weightings(Qfb(:,:,mode),Rfb(:,:,mode),Sfb(:,:,mode),Qfbf(:,:,mode));
end

%% set reference for quadratic cost
g        =  [0, -9.81]'; % gravity vector
GRF      =  -WBMC.robotMass *g;
WBDesiredFState     =  repmat(struct('x', zeros(14, 1),...
                                     'u', zeros(4,1),...
                                     'y', [GRF;GRF]), [4,1]);

WBDesiredFState(1).x = [[0,-0.1432,-pi/25,0.35*pi,-0.65*pi,0.35*pi,-0.6*pi]';
                         params.vd;1;zeros(5,1)];
WBDesiredFState(2).x = [[0,-0.1418,pi/35,0.2*pi,-0.58*pi,0.25*pi,-0.7*pi]';
                         params.vd;-1;zeros(5,1)];
WBDesiredFState(3).x = [[0,-0.1325,-pi/40,0.33*pi,-0.48*pi,0.33*pi,-0.75*pi]';
                         params.vd;1;zeros(5,1)];
WBDesiredFState(4).x = [[0,-0.1490,-pi/25,0.35*pi,-0.7*pi,0.25*pi,-0.60*pi]';
                         params.vd;-1;zeros(5,1)];
          
WBDesiredRState.x = [0,-0.13,0,0.3*pi,-0.7*pi,0.3*pi,-0.7*pi, params.vd, 0, 0,0,0,0,0]';
WBDesiredRState.u = zeros(4,1);
WBDesiredRState.y = [GRF;GRF];

for mode = 1:4
    WBDesiredState(mode).x = [WBDesiredRState.x, WBDesiredFState(mode).x];
    WBDesiredState(mode).u = [WBDesiredRState.u, WBDesiredFState(mode).u];
    WBDesiredState(mode).y = [WBDesiredRState.y, WBDesiredFState(mode).y];
end

FBDesiredState = struct('x', [0, -0.15, 0, params.vd, 0, 0]',... 
                        'u', [GRF;GRF],...
                        'y', zeros(4,1));          
for mode = 1:4
    WBPhases(mode).set_reference(WBDesiredState(mode));
    FBPhases(mode).set_reference(FBDesiredState);
end

%% Initialize AL and ReB parameters
WB_AL_ReB_params = repmat(struct('sigma',[],...
                                 'lambda',[],...
                                 'delta',[],...
                                 'eps_ReB',[],...
                                 'eps_smooth',1),[4,1]);
% Stance phases do not have terminal constraint. AL params are empty                          
% Ineq cq, cu, and cy
WB_AL_ReB_params(1).delta   = [0.1*ones(1,2*4), 0.1*ones(1,2*4), 0.1*ones(1,3), 0.005,0.005];
WB_AL_ReB_params(1).eps_ReB = [0.01*ones(1,2*4),  0*ones(1,2*4), 0.01*ones(1,3), 1e-3, 1e-3];

WB_AL_ReB_params(3).delta   = [0.1*ones(1,2*4), 0.1*ones(1,2*4), 0.1*ones(1,3), 0.005,0.005];
WB_AL_ReB_params(3).eps_ReB = [0.01*ones(1,2*4),  0*ones(1,2*4), 0.01*ones(1,3), 1e-3, 1e-3];

% Flight phase needs terminal constraint at TD
WB_AL_ReB_params(2).sigma   = 5;
WB_AL_ReB_params(2).lambda  = 0;
WB_AL_ReB_params(2).delta   = [0.1*ones(1,2*4), 0.1*ones(1,2*4), 0.005,0.005];
WB_AL_ReB_params(2).eps_ReB = [0.01*ones(1,2*4),0*ones(1,2*4), 1e-3, 1e-3];

WB_AL_ReB_params(4).sigma   = 5;
WB_AL_ReB_params(4).lambda  = 0;
WB_AL_ReB_params(4).delta   = [0.1*ones(1,2*4), 0.1*ones(1,2*4), 0.005,0.005];
WB_AL_ReB_params(4).eps_ReB = [0.01*ones(1,2*4),0*ones(1,2*4),1e-3, 1e-3];

FB_AL_ReB_params = struct('sigma',[],...
                          'lambda',[],...
                          'delta',[],...
                          'eps_ReB',[],...
                          'eps_smooth',1);
FB_AL_ReB_params.delta = [0.005, 0.005];
FB_AL_ReB_params.eps_ReB = [1e-3, 1e-3];
for mode = 1:4
    % Initialize AL_ReB_params
    % Default initialization if no input arguments
    % If constraint handle is not specified and there is no input
    % arguements, constraint handle and associated param would be empty
    WBPhases(mode).set_AL_ReB_params(WB_AL_ReB_params(mode));
    FBPhases(mode).set_AL_ReB_params(); 
end
end