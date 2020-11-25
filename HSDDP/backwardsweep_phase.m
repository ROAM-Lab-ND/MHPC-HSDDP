function success = backwardsweep_phase(Phase, T, G_prime, H_prime, dV_prime, regularization)
    % G_prime, H_prime, gradient and hessian of value function at the initial state of next phase
    % T: trajectory around which backward sweep is performed
    N_horizon = length(T.Ubar);
    T.dV = dV_prime;

    % impact-aware DDP step
    Px = Phase.resetmap_par(T.X(:,end));
    [T.G(:,end), T.H(:,:,end)] = impact_aware_step(T.phiInfo, Px, G_prime, H_prime);

    success = 1;

    for k = N_horizon-1:-1:1
        % Gradient and Hessian of value approx at next step
        Gk_next = T.G(:,k+1);
        Hk_next = T.H(:,:,k+1);

        % Dynamics linearization at current step
        Ak = T.dynInfo(k).fx; Bk = T.dynInfo(k).fu;
        Ck = T.dynInfo(k).gx; Dk = T.dynInfo(k).gu;
       
        % Compute Q info
        Qx  = T.lInfo(k).lx  + Ak'*Gk_next         + Ck'*T.lInfo(k).ly;
        Qu  = T.lInfo(k).lu  + Bk'*Gk_next         + Dk'*T.lInfo(k).ly;
        Qxx = T.lInfo(k).lxx + Ck'*T.lInfo(k).lyy*Ck  + Ak'*Hk_next*Ak;
        Quu = T.lInfo(k).luu + Dk'*T.lInfo(k).lyy*Dk  + Bk'*Hk_next*Bk;
        Qux = T.lInfo(k).lux + Dk'*T.lInfo(k).lyy*Ck  + Bk'*Hk_next*Ak;

        % regularization
        Qxx = Qxx + eye(Phase.model.xsize)*regularization;
        Quu = Quu + eye(Phase.model.usize)*regularization;

        [~, p] = chol(Quu-eye(Phase.model.usize)*1e-9);
        if p ~= 0
            success = 0;
            break;
        end

        % Standard equations
        Quu_inv     = Sym(eye(Phase.model.usize)/Quu);
        T.dU(:,k)   = -Quu_inv*Qu;
        T.K(:,:,k)  = -Quu_inv*Qux;
        T.G(:,k)    = Qx  - (Qux')*(Quu_inv* Qu );
        T.H(:,:,k)  = Qxx - (Qux')*(Quu_inv* Qux);
        T.dV        = T.dV - Qu'*(Quu\Qu);

        if any(isnan(T.dU(:,k))) || any(isnan(T.G(:,k)))
            success = 0;
            error('error: du or Vx has nan at k = %d', k);
%             break;
        end

    end
end