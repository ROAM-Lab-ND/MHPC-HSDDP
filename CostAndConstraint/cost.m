classdef cost < handle
    properties
        WBModel
        FBModel       
    end
    
    methods % constructor
        function co = cost(wbmodel, fbmodel)
            co.WBModel = wbmodel;
            co.FBModel = fbmodel;           
        end
    end
    
    methods
        function lInfo = running_cost_Info(co,x,xd,u,ud,y,yd,Q,R,S,dt)
            lInfo = lInfoStruct(length(x),length(u),length(y));
            lInfo.l     = 2*dt*1/2*((x-xd)'*Q*(x-xd) + (u-ud)'*R*(u-ud) + (y-yd)'*S*(y-yd));
            lInfo.lx    = 2*dt*Q*(x-xd); % column vec
            lInfo.lu    = 2*dt*R*(u-ud);
            lInfo.ly    = 2*dt*S*(y-yd);
            lInfo.lxx   = 2*dt*Q;
            lInfo.lux   = zeros(length(u),length(x));
            lInfo.luu   = 2*dt*R;
            lInfo.lyy   = 2*dt*S;
        end
        
        function phiInfo = terminal_cost_Info(co,x,xd,Q)
            phiInfo.phi = 1/2*(x-xd)'*Q*(x-xd);
            phiInfo.phix = Q*(x-xd);
            phiInfo.phixx = Q;
        end
        
        function phiInfo = jump_terminal_cost_Info(co, x, xd, Q)
            phiInfo = co.terminal_cost_Info(x,xd,Q);
            
            % penalize distance from foot to gap function
            link = [3, 5];
            for idx = 1:2
                p = co.WBModel.getPosition(x, link(idx), [0,-co.WBModel.kneeLinkLength]');
                [J,~] = co.WBModel.getJacobian(x, link(idx), [0,-co.WBModel.kneeLinkLength]');
                [Jx,~] = co.WBModel.getJacobianPar(x, link(idx), [0,-co.WBModel.kneeLinkLength]');
                [g,dg,ddg] = gapFunc(p(1));
                d = p(2) - g;
                dx = [J(2,:), zeros(1,co.WBModel.qsize)] - [dg*J(1,:),zeros(1,co.WBModel.qsize)];
                dxx = [squeeze(Jx(2,:,:));zeros(co.WBModel.qsize, co.WBModel.xsize)] - ...
                    [squeeze(Jx(1,:,:));zeros(co.WBModel.qsize, co.WBModel.xsize)]*dg -...
                    blkdiag(J(1,:)'*J(1,:),zeros(co.WBModel.qsize))*ddg;
                a = -40;
                phi = exp(a*d);
                phix = a*phi*dx';
                phixx = a*phi*dxx + a^2*phi*(dx'*dx);
                
                phiInfo = co.add_terminal_cost(phiInfo,phi,phix,phixx);
            end
            
        end
    end
    
    methods (Static)
        function phiInfo = add_terminal_cost(phiInfo,phi,phix,phixx)
            phiInfo.phi = phiInfo.phi + phi;
            phiInfo.phix = phiInfo.phix + phix;
            phiInfo.phixx = phiInfo.phixx + phixx;
        end
    end
end