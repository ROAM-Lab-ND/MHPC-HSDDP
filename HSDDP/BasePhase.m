classdef BasePhase < matlab.mixin.Copyable
    properties    
        model
        mode {mustBePositive, mustBeInteger} = 1;
        next_mode {mustBePositive, mustBeInteger} = 2;
        AL_ReB_params %(penalty, Lagrange, relaxation, ReB weighting)
        n_eq
        n_ineq       
        termconstr_active = 0;
        ineqconstr_active = 0;
        Td      % desired trajectory. Could be either time-series data or constant value 
        dt 
        time
        model_transition_flag = 0;
        Pr
        hierarchy
        lastPhase = 0;
    end
    
    properties % weightings of quadratic cost
        Q
        R 
        S 
        Qf
    end
    
    properties
        termconstr_handle
        ineqconstr_handle
        running_cost_handle
        terminal_cost_handle
    end
    
    methods % Constructor        
        function Ph = BasePhase(model, mode, hierarchy)            
           if nargin > 0
               Ph.Initialization(model, mode, hierarchy);
           end           
        end
        
        function Initialization(Ph, model, mode, hierarchy)
            Ph.model = model;
            Ph.mode  = mode;
            Ph.hierarchy = hierarchy;
            Ph.Q = eye(model.xsize);
            Ph.R = eye(model.usize);
            Ph.S = zeros(model.ysize);
            Ph.Qf = 50*eye(model.xsize);
            Ph.dt = model.dt;            
        end
    end
    
    methods % dynamics
        function [x_next, y] = dynamics(Ph,x,u)
           [x_next, y] = Ph.model.dynamics(x,u,Ph.mode);            
        end
        
        function [x_next, y] = resetmap(Ph,x)
            [x_next, y] = Ph.model.resetmap(x,Ph.mode,Ph.next_mode);
            if Ph.model_transition_flag
                Ph.Pr = [eye(3), zeros(3,11);
                      zeros(3,7),eye(3),zeros(3,4)];
                x_next = Ph.Pr*x_next;
            else
                Ph.Pr = eye(Ph.model.xsize);
            end
        end
        
        function dynInfo     = dynamics_par(Ph,x,u)
            dynInfo = Ph.model.dynamics_par(x,u,Ph.mode);
        end
        
        function Px          = resetmap_par(Ph,x)
            Px = Ph.model.resetmap_par(x,Ph.mode,Ph.next_mode); 
            if isempty(Ph.Pr)
                Ph.Pr = eye(Ph.model.xsize);
            end
            Px = Ph.Pr*Px;
        end                                           
    end
    
    methods % cost
        function lInfo       = running_cost_Info(Ph,k,x,u,y)
            if size(Ph.Td.x,2)<= 2
                xd = Ph.Td.x(:,1);                            
            else
                xd = Ph.Td.x(:,k);                
            end
            
            if size(Ph.Td.u,2)<= 2
                ud = Ph.Td.u(:,1);
            else
                ud = Ph.Td.u(:,k);
            end
            
            if size(Ph.Td.y,2)<= 2
                yd = Ph.Td.y(:,1);                
            else
                yd = Ph.Td.y(:,k);
            end
            
            lInfo = Ph.running_cost_handle(x,xd,u,ud,y,yd,Ph.Q,Ph.R,Ph.S,Ph.dt);
        end
        
        function phiInfo     = terminal_cost_Info(Ph, x)
            xd = Ph.Td.x(:,end);
            scale = 1;
            if Ph.lastPhase
                scale = 10;
            end
            phiInfo = Ph.terminal_cost_handle(x,xd,10*Ph.Qf);
        end
    end
    
    methods % constraint
        function [h, hx, hxx]= terminal_constr_Info(Ph, x)
            if Ph.termconstr_active
                message = sprintf('Terminal constraint is not set for mode %d',Ph.mode);
                assert(~isempty(Ph.termconstr_handle),message);
                [h, hx, hxx] = Ph.termconstr_handle(x,Ph.mode);  
            else
                h = [];
            end                                   
        end        
        
        function ineqInfo    = ineq_constr_Info(Ph, x, u, y) 
            if Ph.ineqconstr_active
                message = sprintf('Ineq constraint is not set for mode %d', Ph.mode);
                assert(~isempty(Ph.ineqconstr_handle), message);  
                ineqInfo = Ph.ineqconstr_handle(x, u, y, Ph.mode); 
            else
                ineqInfo = [];
            end                          
        end
    end
    
    methods % helpful functions
        function set_weightings(Ph,Q,R,S,Qf)
            Ph.Q = Q; Ph.R = R; Ph.S = S; Ph.Qf = Qf;
        end
        
        function set_constraint_active(Ph, constrType)
            switch constrType
                case 'terminal'
                    Ph.termconstr_active = 1;
                case 'ineq'
                    Ph.ineqconstr_active = 1;
                otherwise
                    error('Nonexist constraint type. Constraint type needs to be terminal or ineq.');                    
            end            
        end
        
        function set_terminal_constraint(Ph, termconstrFunc)
            Ph.termconstr_handle = termconstrFunc;            
        end
        
        function set_ineq_constraint(Ph, ineqconstrFunc)
            Ph.ineqconstr_handle = ineqconstrFunc;
        end
        
        function set_running_cost(Ph, runningCostFunc)
            Ph.running_cost_handle = runningCostFunc;
        end
        
        function set_terminal_cost(Ph,terminalCostFunc)
            Ph.terminal_cost_handle = terminalCostFunc;
        end
        
        function set_reference(Ph, Tdesire)
            Ph.Td = Tdesire;
        end
        
        function set_model_params(Ph, p)           
            Ph.model.Initialize_model(p);
        end
        
        function set_mode(Ph, mode)
            Ph.mode = mode;
        end
        
        function set_time(Ph, time)
            Ph.time = time;
        end
        
        function set_AL_ReB_params(Ph, al_reb_params)
            % Initialize AL ReB params using al_reb_params
            % If al_red_params not assigned, use default options
            if nargin < 2
                x   =  zeros(Ph.model.xsize,1);
                u   =  zeros(Ph.model.usize,1);
                y   =  zeros(Ph.model.ysize,1);
                al_reb_params.sigma  = [];
                al_reb_params.lambda = [];                
                al_reb_params.delta  = [];   
                al_reb_params.eps_ReB = [];
                al_reb_params.eps_smooth = [];
                
                if Ph.termconstr_active
                    hInfo  = Ph.terminal_constr_Info(x);
                    Ph.n_eq   = length(hInfo.h);
                    al_reb_params.lambda = zeros(1, Ph.n_eq);
                    al_reb_params.sigma  = 5;
                end
                
                if Ph.ineqconstr_active
                    cInfo  = Ph.ineq_constr_Info(x,u,y);
                    Ph.n_ineq = length(cInfo.c);
                    al_reb_params.delta  = 0.1*ones(1, Ph.n_ineq);
                    al_reb_params.eps_ReB = ones(1, Ph.n_ineq);
                    al_reb_params.eps_Smooth = 1;
                end   
            else
                if isempty(Ph.termconstr_handle)
                    assert(isempty([al_reb_params.lambda, al_reb_params.sigma]), 'non-empty AL params for empty terminal constraint');
                end
                
                if isempty(Ph.ineqconstr_handle)
                    assert(isempty([al_reb_params.delta, al_reb_params.eps_ReB]), 'non-empty ReB params for empty ineqconstraint');
                end
            end
            Ph.AL_ReB_params = al_reb_params;
        end  
        
        function set_next_mode(Ph, next_mode) 
            Ph.next_mode = next_mode;
        end
        
        function set_model_transition_flag(Ph, flag)
            Ph.model_transition_flag = flag;            
        end
       
    end
                      
end