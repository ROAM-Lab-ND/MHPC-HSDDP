classdef WB_boundingPhase < BasePhase   
    properties
        model
        mode
    end
    
    methods 
        % Constructor
        function Ph = WB_boundingPhase(model, mode)            
            qsize   = 7;
            xsize   = 14;
            usize   = 4;
            ysize   = 4;
            x = zeros(xsize, 1);
            u = zeros(usize, 1);
            y = zeros(ysize, 1);
            [h,~,~] = WB_terminal_constr_Info(x,mode);
            ineq_Info = WB_ineq_constr_Info(x,u,y,mode);
            n_eq = length(h);
            n_ineq = length(ineq_Info.c);
            Ph@BasePhase(qsize, xsize, usize, ysize, n_eq, n_ineq);
            Ph.model = model;
            Ph.mode = mode;
        end
    end
    
    methods 
        function [x_next, y] = dynamics(Ph, x, u) 
            [x_next, y] = Ph.model.dynamics(x, u, Ph.mode);
        end
        function dynInfo     = dynamics_par(Ph, x, u)
            dynInfo = Ph.model.dynamics_par(x, u, Ph.mode);
        end
        function [x_next, y] = resetmap(Ph, x) 
            [x_next, y] = Ph.model.resetmap(x, Ph.mode);
        end
        function Px          = resetmap_par(Ph, x) 
            Px = Ph.model.resetmap_par(x, Ph.mode);
        end
        function lInfo       = running_cost_Info(Ph, x, u, y)
            lInfo = WB_running_cost_Info(x, u, y, Ph.mode);
        end
        function phiInfo     = terminal_cost_Info(Ph, x)
            phiInfo = WB_terminal_cost_Info(x, Ph.mode);
        end
        function [h, hx, hxx]= terminal_constr_Info(Ph, x)
            [h, hx, hxx] = WB_terminal_constr_Info(x, Ph.mode);
        end
        function ineqInfo    = ineq_constr_Info(Ph, x, u, y)  
            ineqInfo = WB_ineq_constr_Info(x,u,y,Ph.mode);
        end
    end    
    
    methods 
        function resetDynamics(Ph)
        end
    end

end