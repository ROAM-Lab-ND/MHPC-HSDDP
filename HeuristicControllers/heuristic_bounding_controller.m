function heuristic_bounding_controller(model, modeSeq, hybridT)
%% flight phase reference angles
the_ref = [pi/4,-pi*7/12,pi/4,-pi*7/12]';
NomSpringLen = 0.2462;
kp = 5*diag([8,1,12,10]);
kd = 1*diag([1,1,1,1]);
Kspring = 2200;         % spring stiffness


for idx = 1:length(modeSeq)
    mode = modeSeq(idx);    
    for k = 1:hybridT(idx).N_horizon - 1
        xk  =   hybridT(idx).Xbar(:,k);
        qk  =   xk(1:model.qsize,1);
        switch mode
            case 1 % bs
                [Jc,~] = model.getFootJacobian(xk, mode);
                Jc(:,1:3) = [];
                leg = 2;
                v = getSpringVec(qk,model,leg);
                Fk = -v/norm(v)*Kspring*(norm(v)-NomSpringLen);
                uk = Jc'*Fk*3;
                if idx > find(modeSeq==1,1)
                    uk = Jc'*Fk*1.7;
                end
            case 3 % fs
                [Jc,~] = model.getFootJacobian(xk, mode);
                Jc(:,1:3) = [];
                leg = 1;
                v = getSpringVec(qk,model,leg);
                Fk = -v/norm(v)*Kspring*(norm(v)-NomSpringLen);
                uk = Jc'*Fk*2.2;
                if idx > find(modeSeq==3,1)
                    uk = Jc'*Fk*1.4;
                end
            case {2,4}
                the_k = xk(4:7,1);
                thed_k = xk(11:end,1);
                uk = kp*(the_ref-the_k) - kd*thed_k; % first flight
        end
        [xk_next, yk] = model.dynamics(xk, uk, mode);
        hybridT(idx).Xbar(:, k+1) = xk_next;
        hybridT(idx).Ubar(:, k)   = uk;
        hybridT(idx).Y(:, k)      = yk;
    end
    hybridT(idx).Ubar(:, end)    = hybridT(idx).Ubar(:, end - 1);    
    if idx < length(modeSeq)
        [xk_next, yk] = model.resetmap(hybridT(idx).Xbar(:,end), mode, modeSeq(idx+1));
        hybridT(idx+1).Xbar(:, 1) = xk_next;
    end
    hybridT(idx).Y(:, end)       = yk;
end
end

