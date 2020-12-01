classdef PhaseTrajectory < handle
    properties
        N_horizon
        Ubar
        Xbar
        U
        X
        Y                
        G
        H
        K
        dU
        V
        dV
        lInfo
        dynInfo
        phiInfo
    end
    
    methods 
        function T = PhaseTrajectory(xsize, usize, ysize, N_horizon)
            if nargin > 0
                T.Initialization(xsize, usize, ysize, N_horizon);
            end
        end
        
        function Initialization(T,xsize, usize, ysize, N_horizon)
           T.Ubar   = zeros(usize, N_horizon);
           T.Xbar   = zeros(xsize, N_horizon);
           T.U      = zeros(usize, N_horizon);
           T.dU     = zeros(usize, N_horizon);
           T.X      = zeros(xsize, N_horizon);
           T.Y      = zeros(ysize, N_horizon);
           T.G      = zeros(xsize, N_horizon);
           T.H      = zeros(xsize, xsize, N_horizon);
           T.K      = zeros(usize, xsize, N_horizon);           
           T.V      = 0;
           T.dV     = 0;
           T.lInfo  = repmat(struct('l',    0, ... 
                                    'lx',   zeros(xsize,1), ...
                                    'lu',   zeros(usize,1), ...
                                    'ly',   zeros(ysize,1), ...
                                    'lxx',  zeros(xsize, xsize), ...
                                    'lux',  zeros(usize, xsize), ...
                                    'luu',  zeros(usize, usize), ...
                                    'lyy',  zeros(ysize, ysize)), [N_horizon, 1]);
           T.phiInfo.phi = struct('phi',    0, ...
                                  'phix',   zeros(xsize, 1), ...
                                  'phixx',  zeros(xsize, xsize));
           T.dynInfo     = repmat(struct('fx', zeros(xsize, xsize), ...
                                         'fu', zeros(xsize, usize), ...
                                         'gx', zeros(ysize, xsize), ...
                                         'gu', zeros(ysize, usize)),[N_horizon, 1]); 
           T.N_horizon = N_horizon;
        end
        
        function equal = compare(T, T2)
            equal = 1;
            if ~all(size(T.Ubar), size(T2.Ubar))
                equal = 0;
                return
            end
            
            if ~all(size(T.Xbar), size(T2.Xbar))
                equal = 0;
                return
            end
            
            if ~all(size(T.U), size(T2.U))
                equal = 0;
                return
            end
            
            if ~all(size(T.X), size(T2.X))
                equal = 0;
                return
            end
            
            if ~all(size(T.H), size(T2.H))
                equal = 0;
                return
            end
            
            if ~all(size(T.G), size(T2.G))
                equal = 0;
                return
            end
            
            if ~all(size(T.K), size(T2.K))
                equal = 0;
                return
            end
        end
        
        function updateNominal(T)
            T.Xbar = T.X;
            T.Ubar = T.U;
        end
        
        function set_nom_initial_condition(T, x0)
            T.Xbar(:,1) = x0;
        end
        
        function set_act_initial_condition(T, x0)
            T.X(:,1) = x0;
        end
    end
end