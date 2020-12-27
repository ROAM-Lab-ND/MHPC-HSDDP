classdef cost < handle
    properties
        WBModel
        FBModel
        joint_virtualLeg
    end
    
    methods % constructor
        function co = cost(wbmodel, fbmodel)
            co.WBModel = wbmodel;
            co.FBModel = fbmodel;           
            co.joint_virtualLeg = [0.3*pi,-0.7*pi,0.3*pi,-0.7*pi]';
        end
    end
    
    methods
        function lInfo = running_cost_Info(co,x,xd,u,ud,y,yd,Q,R,S,dt,flag)
            switch flag
                case 'nopar'
                    lInfo = 2*dt*1/2*((x-xd)'*Q*(x-xd) + (u-ud)'*R*(u-ud) + (y-yd)'*S*(y-yd));
                case 'par'
                    lInfo = lInfoStruct(length(x),length(u),length(y));
                    lInfo.l     = 2*dt*1/2*((x-xd)'*Q*(x-xd) + (u-ud)'*R*(u-ud) + (y-yd)'*S*(y-yd));
                    lInfo.lx    = 2*dt*Q*(x-xd); % column vec
                    lInfo.lu    = 2*dt*R*(u-ud);
                    lInfo.ly    = 2*dt*S*(y-yd);
                    lInfo.lxx   = 2*dt*Q;
                    lInfo.lux   = zeros(length(u),length(x));
                    lInfo.luu   = 2*dt*R;
                    lInfo.lyy   = 2*dt*S;
                otherwise
                    error('Wrong input! flag must be either "nopar" or "par"');
            end                            
        end
        
        function phiInfo = terminal_cost_Info(co,x,xd,Q,flag)
            switch flag
                case 'nopar'
                    phiInfo = 1/2*(x-xd)'*Q*(x-xd);
                case 'par'
                    phiInfo.phi = 1/2*(x-xd)'*Q*(x-xd);
                    phiInfo.phix = Q*(x-xd);
                    phiInfo.phixx = Q;
                otherwise
                    error('Wrong input! flag must be either "nopar" or "par"');
            end
            
        end
        
        function phiInfo = WB_jump_terminal_cost_Info(co, x, xd, Q, flag)
            phiInfo = co.terminal_cost_Info(x,xd,Q, flag);
            a = -5;
            s = 5;
            % penalize distance from foot to gap function
            link = [3, 5];
            for idx = 1:2
                p = co.WBModel.getPosition(x, link(idx), [0,-co.WBModel.kneeLinkLength]');
                [g,dg,ddg] = gapFunc(p(1));
                d = p(2) - g;
                if d < -0.05 % If the foot is too lower than gap function, skip
                    break;
                end
                
                phi = s*exp(a*d);
                if strcmp(flag, 'nopar')
                    phiInfo = phiInfo + phi;
                else
                    [J,~] = co.WBModel.getJacobian(x, link(idx), [0,-co.WBModel.kneeLinkLength]');
                    [Jx,~] = co.WBModel.getJacobianPar(x, link(idx), [0,-co.WBModel.kneeLinkLength]');
                    dx = [J(2,:), zeros(1,co.WBModel.qsize)] - [dg*J(1,:),zeros(1,co.WBModel.qsize)];
                    dxx = [squeeze(Jx(2,:,:));zeros(co.WBModel.qsize, co.WBModel.xsize)] - ...
                          [squeeze(Jx(1,:,:));zeros(co.WBModel.qsize, co.WBModel.xsize)]*dg -...
                           blkdiag(J(1,:)'*J(1,:),zeros(co.WBModel.qsize))*ddg;
                    phix = s*a*phi*dx';
                    phixx = s*(a*phi*dxx + a^2*phi*(dx'*dx));
                    phiInfo = co.add_terminal_cost(phiInfo,phi,phix,phixx);
                end                                                
            end
            
        end
        
        function phiInfo = FB_jump_terminal_cost_Info(co, xFB, xdFB, Q, flag)
            
            phiInfo = co.terminal_cost_Info(xFB,xdFB,Q, flag);
            
            % Append virtual legs to the floating base
            qWB = [xFB(1:3); co.joint_virtualLeg];
            xWB = [qWB; zeros(7,1)];
            a = -5;
            s = 5;
            % penalize distance from foot to gap function
            link = [3, 5];
            for idx = 1:2
                p = co.WBModel.getPosition(qWB, link(idx), [0,-co.WBModel.kneeLinkLength]');
                [g,dg,ddg] = gapFunc(p(1));
                d = p(2) - g;
                if d < -0.05 % If the foot is too lower than gap function, skip
                    break;
                end
                
                phi = s*exp(a*d);
                if strcmp(flag, 'nopar')
                    phiInfo = phiInfo + phi;
                else
                    [J_WB,~] = co.WBModel.getJacobian(xWB, link(idx), [0,-co.WBModel.kneeLinkLength]');
                    [Jx_WB,~] = co.WBModel.getJacobianPar(xWB, link(idx), [0,-co.WBModel.kneeLinkLength]');
                    dx = [J_WB(2,1:3), zeros(1,co.FBModel.qsize)] - [dg*J_WB(1,1:3),zeros(1,co.FBModel.qsize)];
                    dxx = blkdiag(squeeze(Jx_WB(2,1:3,1:3)), zeros(co.FBModel.qsize)) - ...
                          blkdiag(squeeze(Jx_WB(1,1:3,1:3)), zeros(co.FBModel.qsize))*dg -...
                          blkdiag(J_WB(1,1:3)'*J_WB(1,1:3),zeros(co.FBModel.qsize))*ddg;
                    phix = s*a*phi*dx';
                    phixx = s*(a*phi*dxx + a^2*phi*(dx'*dx));
                    phiInfo = co.add_terminal_cost(phiInfo,phi,phix,phixx);
                end
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