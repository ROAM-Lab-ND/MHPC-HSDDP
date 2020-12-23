classdef mhpcController < RobotController
    properties
        xopt  % optimal state given by HSDDP
        uopt  % optimal feedforward by HSDDP
        Kopt  % optimal feedback gain by HSDDP
        delay
    end
    
    properties
        HSDDP
        HybridTrajectory
        WBModel
        FBModel
        WBPhases
        FBPhases
        gait
        problem_data
    end
    
    methods % constructor
        function C = mhpcController(WBModel, FBModel, gait, Problem_data, name)
            if nargin > 0
                initialization(C, WBModel, FBModel, gait, Problem_data,name);
            end
        end
                      
        function initialization(C,WBModel, FBModel, gait, Problem_data,name)
            C.WBModel = WBModel;
            C.FBModel = FBModel;
            C.gait = gait;                       
            C.problem_data = Problem_data;
            if strcmp(name, 'bounding')
                [C.WBPhases, C.FBPhases] = ConstructParentPhases(WBModel, FBModel,Problem_data);
            elseif strcmp(name, 'jumping')
                [C.WBPhases, C.FBPhases] = ConstructParentPhasesJumping(WBModel, FBModel,Problem_data);
            end            
        end
        
        function initializeHSDDP(C)
            %% Preallocate HSDDP phases
            Phases = repmat(BasePhase(), [C.problem_data.n_Phases, 1]);
            
            % Allocate WB (whole-body) phases
            for idx = 1:C.problem_data.n_WBPhases
                Phases(idx) = copy(C.WBPhases(C.problem_data.phaseSeq(idx)));
                Phases(idx).set_next_mode(C.problem_data.phaseSeq(idx+1))
                Phases(idx).set_time(C.problem_data.t_horizons(idx)); % set time of period for each phase
                Phases(idx).set_model_transition_flag(0);
            end
            if C.problem_data.n_WBPhases > 0
                Phases(C.problem_data.n_WBPhases).set_model_transition_flag(1);
            end
            
            % Allocate FB (floating-base) phases
            for idx = C.problem_data.n_WBPhases+1:C.problem_data.n_Phases
                Phases(idx) = copy(C.FBPhases(C.problem_data.phaseSeq(idx)));
                Phases(idx).set_next_mode(C.problem_data.phaseSeq(idx+1))
                Phases(idx).set_time(C.problem_data.t_horizons(idx));
            end
            
            %% Preallocate memory for trajectories of each phase
            C.HybridTrajectory = repmat(PhaseTrajectory(),[C.problem_data.n_Phases, 1]);
            for idx = 1:C.problem_data.n_Phases
                C.HybridTrajectory(idx) = PhaseTrajectory(Phases(idx).model.xsize,...
                    Phases(idx).model.usize,...
                    Phases(idx).model.ysize,...
                    C.problem_data.N_horizons(idx));
            end
            
            %% Create foldhold planner
            FootPlanner = FootholdPlanner(C.WBModel, C.gait, C.problem_data.phaseSeq(C.problem_data.n_WBPhases+1), ...
                C.problem_data.n_FBPhases, C.problem_data.t_horizons(C.problem_data.n_WBPhases+1:end), C.problem_data.vd);
            
            %%
            C.HSDDP = HybridSystemsDDP(Phases, C.HybridTrajectory);
            C.HSDDP.set_FootPlanner(FootPlanner);
        end
        
        function runHSDDP(C, x0, options)
            C.initializeHSDDP();
            
            C.HybridTrajectory(1).set_nom_initial_condition(x0);
            
            heuristic_bounding_controller(C.WBModel, C.problem_data.phaseSeq(1:C.problem_data.n_WBPhases), C.HybridTrajectory(1:C.problem_data.n_WBPhases));
                        
            C.HSDDP.set_initial_condition(x0);
                        
            % set lastPhase flag to 1 for the last phase
            C.HSDDP.Phases(end).lastPhase = 1;
            
            % update desired terminal state at each phase
            for idx = 1:C.HSDDP.n_Phases
                C.HSDDP.Phases(idx).Td.x(1,end) =  x0(1) + C.problem_data.vd*sum(C.problem_data.t_horizons(1:idx));
            end
            
            [C.xopt, C.uopt, C.Kopt] = C.HSDDP.Run(options);
        end
        
        function updateHSDDP(C, problem_data)
            C.problem_data = problem_data;
        end
        
        function InformControllerDelay(C, delay)
            C.delay = delay;
            C.xopt{1}(:,1:delay) = [];
            C.uopt{1}(:,1:delay) = [];
            C.Kopt{1}(:,:,1:delay) = [];
        end
        
        function uk = run(C,xk,k,pidx)
            uk = C.uopt{pidx}(:,k) + C.Kopt{pidx}(:,:,k)*(xk - C.xopt{pidx}(:,k));
        end
    end
    
end