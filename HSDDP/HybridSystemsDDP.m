classdef HybridSystemsDDP < handle
    properties %(Access = private)
        x0
        n_Phases      
        N_horizons
        Phases
        hybridT     % array of phase trajectory
    end
    
    properties
        V = 0;
        dV = 0;
        h;      % terminal constraint
        hnorm = 0;  % total violation evaluated in 2norm
    end
    
    properties
        FootPlanner
    end
    
    methods %(constructor)
        function DDP = HybridSystemsDDP(Phases, HybridT)
            if nargin > 0
               DDP.Initialization(Phases, HybridT);
            end
        end
        
        function Initialization(DDP, Phases, HybridT)
            assert(length(Phases) == length(HybridT), 'Fail to construct HSDDP. Dimensions of Phases and HybridTrajectories do not match.');
            DDP.Phases  = Phases;
            DDP.n_Phases = length(Phases);
            DDP.N_horizons = zeros(1, DDP.n_Phases);
            for idx = DDP.n_Phases
                DDP.N_horizons(idx) = HybridT(idx).N_horizon;
            end
            DDP.h       = cell(DDP.n_Phases,1);
            DDP.hybridT = HybridT;            
        end
    end
    
    methods
        function success = forwardsweep(DDP, eps, AL_ReB_params, options)
            DDP.V = 0;
            DDP.hnorm = 0;
            hnormsqure = 0;
            success = 1;
            Footholds = [];
            for idx = 1:DDP.n_Phases                
                if idx == 1 
                    % first phase                    
                    if strcmp(DDP.Phases(idx).hierarchy, 'simple')
                        Pr = [eye(3), zeros(3,11);
                              zeros(3,7),eye(3),zeros(3,4)];
                        initialState = Pr*DDP.x0;  
                    else
                        initialState = DDP.x0;
                    end
                    
                    DDP.hybridT(idx).set_nom_initial_condition(initialState);
                    DDP.hybridT(idx).set_act_initial_condition(initialState);
                else
                    % reset 
                    initialState = DDP.Phases(idx-1).resetmap(DDP.hybridT(idx-1).X(:,end));
                    DDP.hybridT(idx).set_act_initial_condition(initialState);
                end
                
                % execute foothold planner in the first simple model phase   
                firstSimple = find(strcmp({DDP.Phases.hierarchy},'simple'),1);
                if idx == firstSimple
                    Footholds = DDP.FootPlanner.getFoothold(initialState);
                end                                
                                
                if strcmp(DDP.Phases(idx).hierarchy, 'simple')
                    DDP.Phases(idx).set_model_params(Footholds(:,idx - (firstSimple-1)));
                end
                [DDP.h{idx}, success] = forwardsweep_phase(DDP.Phases(idx), DDP.hybridT(idx), eps, AL_ReB_params(idx), options);
                if ~success
                    break;
                end
                hnormsqure = hnormsqure + DDP.h{idx}*DDP.h{idx}';
                DDP.V = DDP.V + DDP.hybridT(idx).V;
            end 
            DDP.hnorm = sqrt(hnormsqure);
        end
        
        function forwarditeration(DDP, AL_ReB_params, options)
            eps = 1;
            Vprev = DDP.V;
            
            while eps > 1e-10                                                                 
                 success = DDP.forwardsweep(eps, AL_ReB_params, options);
                 
                 if options.Debug
                    fprintf('\t eps=%.3e \t cost change=%.3e \t min=%.3e\n',eps, DDP.V-Vprev, options.gamma* eps*(1-eps/2)*DDP.dV );
                 end
                 
                 % check if V is small enough to accept the step
                 if success && (DDP.V < Vprev + options.gamma*eps*(1-eps/2)*DDP.dV)
                     % if so, break
                     break;
                 end
                 
                 % Elsem backtrack
                 eps = options.alpha*eps;
            end
        end
        
        function success = backwardsweep(DDP, regularization)
            success = 1;
            for idx = DDP.n_Phases:-1:1
                if idx == DDP.n_Phases
                    Gprime = zeros(DDP.Phases(idx).model.xsize, 1);
                    Hprime = zeros(DDP.Phases(idx).model.xsize, DDP.Phases(idx).model.xsize);
                    dVprime = 0;
                else
                    Gprime = DDP.hybridT(idx+1).G(:,1);
                    Hprime = DDP.hybridT(idx+1).H(:,:,1);
                    dVprime = DDP.hybridT(idx+1).dV;
                end
                success = backwardsweep_phase(DDP.Phases(idx), DDP.hybridT(idx), Gprime, Hprime, dVprime, regularization);
                if success == 0
                    break;
                end
            end
            DDP.dV = DDP.hybridT(1).dV;
        end
                        
        function [xopt, uopt, Kopt] = Run(DDP, options)
            % Initialize AL and ReB params
            % @brief AL_ReB_params = struct('sigma', 'lambda', 'delta', 'beta_deta')
            xopt = cell(1,DDP.n_Phases);
            uopt = cell(1,DDP.n_Phases);
            Kopt = cell(1,DDP.n_Phases);
            
            AL_ReB_params = repmat(struct('sigma',[],...
                                          'lambda',[],...
                                          'delta',[],...
                                          'eps_ReB',[],...
                                          'eps_smooth',[]),[1, DDP.n_Phases]);
                                      
            % Initialize AL and ReB parameters with values defined in
            % Parent phases
            for idx = 1:DDP.n_Phases
                AL_ReB_params(idx) = DDP.Phases(idx).AL_ReB_params;
            end
            
            % Initial sweep            
            options.feedback_active = 0;
            DDP.forwardsweep(1, AL_ReB_params, options);
            DDP.updateNominalTrajectory();
            
            % Iterate
            ou_iter = 1;           
                        
            while 1 % Implement AL and ReB in outer loop
                if  options.Debug
                    fprintf('====================================================\n');
                    fprintf('\t Outer loop Iteration %3d\n',ou_iter);
                end
                
                if options.AL_active || options.ReB_active
                    options.feedback_active = 0;
                    DDP.forwardsweep(1, AL_ReB_params, options);
                    DDP.updateNominalTrajectory();
                end
                Vprev = DDP.V;
                regularization = 0;       
                in_iter = 1;
                while 1 % DDP in inner loop
                    if  options.Debug
                        fprintf('================================================\n');
                        fprintf('\t Inner loop Iteration %3d\n',in_iter);
                    end
                    
                    % backward sweep with regularization
                    success = 0;
                    while success == 0
                        if options.Debug
                            fprintf('\t reg=%.3e\n',regularization);
                        end
                        success = DDP.backwardsweep(regularization);
                        
                        if success == 0
                            regularization = max(regularization*options.beta_reg, 1e-3);
                        end
                    end
                    regularization = regularization/20;
                    if regularization < 1e-6
                        regularization = 0;
                    end
                    
                    options.feedback_active = 1;
                    DDP.forwarditeration(AL_ReB_params, options);
                    DDP.updateNominalTrajectory();
                    
                    if (in_iter>=options.max_DDP_iter) || (Vprev - DDP.V < options.DDP_thresh)
                        break
                    end
                    in_iter = in_iter + 1;     
                    Vprev = DDP.V;                    
                end
                fprintf('Total terminal constraint violation %.4f\n',DDP.hnorm);
                if (ou_iter>=options.max_AL_iter) || (DDP.hnorm < options.AL_thresh)
                    break;
                end
                ou_iter = ou_iter + 1;
                AL_ReB_params = DDP.update_AL_ReB_params(AL_ReB_params, DDP.h, options);
            end
                                   
            for idx = 1:DDP.n_Phases
                xopt{idx} = DDP.hybridT(idx).Xbar;
                uopt{idx} = DDP.hybridT(idx).Ubar;
                Kopt{idx} = DDP.hybridT(idx).K;
            end
        end                 
    end
    
    methods
        function set_initial_guess(DDP, Ustart)
            for idx = 1:DDP.n_Phases
                DDP.hybridT(idx).Ubar = Ustart{idx};
            end
        end
        
        function set_hybridTrajectory(DDP, hybridT)
            if compareHybridTrajectory(DDP.hybridT, hybridT)
                DDP.hybridT = hybridT;
            end            
        end
        
        function set_initial_condition(DDP, x0)
            DDP.x0 = x0;
        end
        
        function updateNominalTrajectory(DDP)
            for idx = 1:DDP.n_Phases
                updateNominal(DDP.hybridT(idx));
            end
        end  
        
        function set_FootPlanner(DDP, Planner)
            DDP.FootPlanner = Planner;
        end
    end
    
    methods (Static)
        function params = update_AL_ReB_params(params, h, options)
            for i = 1:length(params)
                
                params(i).lambda = params(i).lambda + h{i}*params(i).sigma;
                params(i).sigma = options.beta_penalty * params(i).sigma;
                
                
                % update ReB params
                if options.beta_relax > 1
                    options.beta_relax = 0.5;
                end
                params(i).delta = options.beta_relax*params(i).delta;
                params(i).delta(params(i).delta<1e-3) = 1e-3;
            end
        end
    end           
end