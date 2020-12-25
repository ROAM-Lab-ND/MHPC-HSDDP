function [h, success] = forwardsweep_phase(Phase, T, eps, AL_ReB_params, options)
    % T: phase trajectory
    % AL_ReB_params: AL and ReB params for T
    success = 1;
    N_horizon = length(T.Ubar);
    T.V = 0;
    h = 0; 
    upre = T.Ubar(:,1);
    
    if options.parCalc_active
            computeFlag = 'par';
        else
            computeFlag = 'nopar';
    end
        
    for k = 1:N_horizon - 1
        ubar    = T.Ubar(:,k);
        xbar    = T.Xbar(:,k);
        x       = T.X(:,k);
        if options.feedback_active
            K       = T.K(:,:,k);
            du      = T.dU(:,k);
            u       = ubar + eps*du + K*(x - xbar);
        else
            u       = ubar;
        end
        if any(isnan(u))
            success = 0;
            break;
        end
        
        [x_next, y] = Phase.dynamics(x, u);
                
        % Running cost info (lInfo_k structure)
        lInfo_k = Phase.running_cost_Info(k, x, u, y, computeFlag);
        
        % Ineq constraint info
        ineqInfo_k = Phase.ineq_constr_Info(x, u, y, computeFlag);

        % Modify lInfo if Reduced Barrier is active
        if options.ReB_active && Phase.ineqconstr_active
            lInfo_k = update_lInfo_with_ReB(lInfo_k,ineqInfo_k, Phase.dt, AL_ReB_params.delta, AL_ReB_params.eps_ReB, computeFlag);
        end

        % Modify lInfo if smoothness penalty is active
        if options.smooth_active
            lInfo_k = update_lInfo_with_Smooth(lInfo_k,u,upre,Phase.dt,AL_ReB_params.eps_smooth, computeFlag);
        end
        
        % update upre
        upre = u;
        
        % Update buffer
        T.X(:,k+1)      = x_next;
        T.Y(:,k)        = y;
        T.U(:,k)        = u;
        
        if options.parCalc_active
            T.lInfo(k)      = lInfo_k;
            T.dynInfo(k)    = Phase.dynamics_par(x, u);
            T.V             = T.V + lInfo_k.l;
        else
            T.V             = T.V + lInfo_k;
        end       
    end
    
    % Compute terminal cost info
    phiInfo = Phase.terminal_cost_Info(T.X(:,end), computeFlag);
    
    if options.AL_active && Phase.termconstr_active
        % Get terminal constraint violation
        [h, hx, hxx] = Phase.terminal_constr_Info(T.X(:,end));
        
        % Update terminal cost if Augmented Lagrangian is active
        phiInfo = update_terminalInfo_with_AL(h, hx, hxx, phiInfo,AL_ReB_params.sigma, AL_ReB_params.lambda, computeFlag);
    end
    
    % Buffer
    if options.parCalc_active
        T.phiInfo = phiInfo;
        T.V = T.V + phiInfo.phi;
    else
        T.V = T.V + phiInfo;
    end
    
end