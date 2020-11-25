classdef BasePhase < handle
    properties
        qsize
        xsize
        usize
        ysize
%         AL_ReB_params %(penalty, Lagrange, relaxation, ReB weighting)
        n_eq
        n_ineq
    end
    
    methods 
        % Constructor
        function Ph = BasePhase(qsize, xsize, usize, ysize, n_eq, n_ineq)
            Ph.qsize = qsize;
            Ph.xsize = xsize;
            Ph.usize = usize;
            Ph.ysize = ysize;
            Ph.n_eq  = n_eq;
            Ph.n_ineq = n_ineq;
        end
    end
    
    methods (Abstract)
        % All member functions below implement nothing
        [x_next, y]     = dynamics(Ph, x, u) 
        dynInfo         = dynamics_par(Ph, x, u)
        [x_next, y]     = resetmap(Ph, x) 
        Px              = resetmap_par(Ph, x) 
        lInfo           = running_cost_Info(Ph, x, u)
        phiInfo         = terminal_cost_Info(Ph, x)
        [h, hx, hxx]    = terminal_constr_Info(Ph, x)
        ineqInfo        = ineq_constr_Info(Ph, x, u, y)                
    end    
    
    methods 
        function resetDynamics(Ph)
        end
    end

%     methods (Static)
%         function valid = check_AL_ReB_Param_Size(Ph)
%             lambda = Ph.AL_ReB_params.lambda;
%             [h,~,~] = Ph.terminal_constr_Info(zeros(Ph.xsize, 1));
%             valid = length(h) == length(lambda);
%         end
%     end
end