function h = forwardsweep_phase(Phase, T, eps, AL_ReB_params, options)
    % T: phase trajectory
    % AL_ReB_params: AL and ReB params for T
    N_horizon = length(T.Ubar);
    T.V = 0;
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
        
        [x_next, y] = Phase.dynamics(x, u);

        % Running cost info (lInfo_k structure)
        lInfo_k = Phase.running_cost_Info(k, x, u, y);

        % Dynamics partials info (dynInfo_k structure)
        dynInfo_k = Phase.dynamics_par(x, u);

        % Ineq constraint info
        ineqInfo_k = Phase.ineq_constr_Info(x, u, y);

        % Modify lInfo if Reduced Barrier is active
        if options.ReB_active && Phase.termconstr_active
            lInfo_k = update_lInfo_with_ReB(lInfo_k,ineqInfo_k, AL_ReB_params.delta, AL_ReB_params.eps_ReB);
        end

        % Modify lInfo if smoothness penalty is active
%         if options.smooth_active
%             lInfo_k = update_lInfo_with_Smooth(lInfo_k,ineq_Info_k,AL_ReB_params.eps_smooth);
%         end

        % Update buffer
        T.X(:,k+1)      = x_next;
        T.Y(:,k)        = y;
        T.U(:,k)        = u;
        T.lInfo(k)      = lInfo_k;
        T.dynInfo(k)    = dynInfo_k;
        T.V             = T.V + lInfo_k.l;
    end
    % Compute terminal cost info
    phiInfo = Phase.terminal_cost_Info(T.X(:,end));
    
    if Phase.termconstr_active
        % Get terminal constraint violation
        [h, hx, hxx] = Phase.terminal_constr_Info(T.X(:,end));
        
        % Update terminal cost if Augmented Lagrangian is active
        phiInfo = update_terminalInfo_with_AL(h, hx, hxx, phiInfo,AL_ReB_params.sigma, AL_ReB_params.lambda);
    else
        h = 0;
    end
    
    % Buffer
    T.phiInfo = phiInfo;
end