classdef FB_boundingPhase < BasePhase   
    properties
        model
        mode
    end
    
    methods 
        % Constructor
        function Ph = FB_boundingPhase(model, mode)            
            qsize   = 3;
            xsize   = 6;
            usize   = 4;
            ysize   = 4;
            Ph@BasePhase(qsize, xsize, usize, ysize);
            Ph.model = model;
            Ph.mode = mode;
        end
    end
    
    methods 
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
end