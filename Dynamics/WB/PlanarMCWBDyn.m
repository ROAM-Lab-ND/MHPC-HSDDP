classdef PlanarMCWBDyn < BaseDyn 
    properties (SetAccess=protected, GetAccess=public)
        dt
        qsize = 7; 
        xsize = 14;
        usize = 4; 
        ysize = 4;   
        mc
        mcParam
        S = [zeros(3, 4);
             eye(4)];
        e = 0;  % elastic coefficient        
    end
    methods 
        % constructor
        function m = PlanarMCWBDyn(dt)
            % Define common abstract properties
            m.dt = dt;            
            % Define properties specific to whole body dynamics
            [m.mc,m.mcParam] = build2DminiCheetah();
        end
    end
    
    methods
        function [x_next, y] = dynamics(m,x,u,mode)
            q   = x(1:7, 1);
            qd  = x(8:14,1);
            
            % Get inertia matrix, and coriolis,centrifugal,gravity marix
            [H, C] = HandC(m.mc, q, qd);                        
            
            % Get contact jacobians
            [J,Jd] = m.getContactJacobian(x, mode);
            
            % assemble KKT matrix
            if isempty(Jd)
                K = H;
                b = m.S*u - C;
            else
                K = [H, J';-J, zeros(size(J,1),size(J,1))];
                b = [m.S*u - C; Jd*qd];
            end
                                    
            % KKT inverse
            f_KKT = K\b;
            
            % continuous dynamics
            fcont = [qd; f_KKT(1:7,1)];
            
            % discrete dynamics
            x_next = x + fcont*m.dt;
            
            % output
            if length(f_KKT) > 7
                ycontact = f_KKT(8:end,1);
            end
            switch mode
                case 1 % BS
                    y = [zeros(2,1); ycontact];
                case {2,4} % FL1 and FL2
                    y = zeros(4,1);
                case 3 % FS
                    y = [ycontact; zeros(2,1)];                                
                case 5 % double stance
                    y = ycontact;
            end            
        end
        
        function [x_next, y] = resetmap(m,x,mode,next_mode)
            qpre = x(1:7,1);        % pre-impact joint angle
            qdpre = x(8:end,1);     % pre-impact joint vel
            
            [H, ~] = HandC(m.mc, qpre, qdpre);
            
            % Get contact Jacobians
            [J, ~] = m.getContactJacobian(x, next_mode);
            
            % Assemble impulse KKT matrix
            if isempty(J)
                K = H;
                b = H*qdpre;
            else
                K = [H, J';-J, zeros(size(J,1),size(J,1))];
                b = [H*qdpre;  m.e*J*qdpre];
            end            
            
            % KKT inverse
            P_KKT = K\b;
            
            % Post state
            qdpost = P_KKT(1:7, 1);
            x_next = [qpre; qdpost];
            
            % Output
            if length(P_KKT) > 7
                impulse = P_KKT(8:end,1);
            end
            switch next_mode
                case 1 % next mode is BS
                    y = [zeros(2,1); impulse];
                case 3 % next mode is FS
                    y = [impulse; zeros(2,1)];                    
                otherwise  
                    % For simplicity, y = 0 at smooth transition. This doesn't affect the
                    % optimization algorithm. An accurate way to do this is
                    % to use the smooth dynamics. Eq(11) in HSDDP paper.
                    y = zeros(4, 1);
            end            
        end
    end
            
    methods
        function dynInfo = dynamics_par(m,x,u,mode)
            q   = x(1:7, 1);
            qd  = x(8:end, 1);
            
            [H,  C] = HandC(m.mc,q,qd);
            
            [J,  Jd] = m.getContactJacobian(x, mode);
            
            [Hx, Cx] = FreeDynamics_par(x);
                                    
            [Jx, Jdx] = m.getContactJacobianPar(x, mode);                                                         
            
            % KKT and KKT partials
            qdx = [zeros(7), eye(7)];
            if isempty(J)
                K  = H;
                b  = m.S*u - C;
                Kx = Hx;
                bx = -Cx;
                bu = m.S*eye(m.usize);
            else
                K = [H, J';-J, zeros(size(J,1),size(J,1))];
                b = [m.S*u - C; Jd*qd];
                Kx = [Hx,  permute(Jx,[2,1,3]);
                    -Jx, zeros(size(Jx,1),size(Jx,1),m.xsize)];
                bx = [ -Cx;
                        getMatVecProdPar(Jd,Jdx,qd,qdx)];
                bu = [m.S*eye(m.usize)
                        zeros(size(Jd,1), m.usize)];
            end
            
            
           % FD partials
           [qddx, qddu, GRFx, GRFu] = getFDPar(K,Kx,b,bx,bu); % g represents GRF if any. For a swing leg gx gu would be empty  
           
           % state-space dynamics partials
           qdu = zeros(7, m.usize);
           fcon_x = [qdx; qddx]; fcon_u = [qdu; qddu];
           dynInfo.fx = eye(m.xsize) + fcon_x*m.dt;
           dynInfo.fu = fcon_u*m.dt;
           gx = zeros(m.ysize, m.xsize);
           gu = zeros(m.ysize, m.usize);           
           % adapt output partial according to current mode
           switch mode
               case 1 % BS
                  gx(3:4, :) = GRFx; gu(3:4, :) = GRFu;
               case 3 % FS
                  gx(1:2, :) = GRFx; gu(1:2, :) = GRFu;               
               case 5 % DS
                  gx = GRFx; gu = GRFu;               
           end
           dynInfo.gx = gx; dynInfo.gu = gu;
        end
        
        function Px = resetmap_par(m,x,mode,next_mode)
            qpre = x(1:7, 1);
            qdpre = x(8:end, 1);
            
            [H,  ~] = HandC(m.mc, qpre, qdpre);
            
            [J,  ~] = m.getContactJacobian(x, next_mode);
            
            [Hx, ~] = FreeDynamics_par(x);
                                    
            [Jx, ~] = m.getContactJacobianPar(x, next_mode);
            
            % KKT and KKT partials
            qdpre_x = [zeros(7), eye(7)];           
            if ~isempty(J)
                K = [H, J';-J, zeros(size(J,1),size(J,1))];
                b = [H*qdpre; m.e*J*qdpre];
                Kx = [Hx,  permute(Jx,[2,1,3]);
                      -Jx, zeros(size(Jx,1),size(Jx,1),m.xsize)];            
                bx = [getMatVecProdPar(H, Hx, qdpre, qdpre_x);
                      getMatVecProdPar(m.e*J,m.e*Jx,qdpre,qdpre_x)];
            else
                K = H;
                b = H*qdpre;
                Kx = Hx;            
                bx = getMatVecProdPar(H, Hx, qdpre, qdpre_x);
            end
                                                                 
            [qdpost_x, ~] = getFDPar(K,Kx,b,bx);
            
            % State-space impact dyanmics partials
            Px = [qdpre_x; qdpost_x];
        end
    end
      
    methods
        function [J,Jd]     = getContactJacobian(m, x, mode)
            % Get contact jacobian and its partial according to mode spec
            J = []; Jd = [];
            linkidx = []; contactLoc = [];            
            switch mode
                case 1
                    linkidx = 5; contactLoc = m.mcParam.kneeLoc;
                case 3
                    linkidx = 3; contactLoc = m.mcParam.kneeLoc; % kneeLoc column vec                
                case {2,4}
                    linkidx = []; contactLoc = [];
                case 5
                    linkidx = [3, 5];
                    contactLoc = [m.mcParam.kneeLoc, m.mcParam.kneeLoc];
            end
            
            for cidx = 1:length(linkidx)
                [J_cidx, Jd_cidx] = getJacobian(x, linkidx(cidx), contactLoc(:,cidx));
                J   = [J; J_cidx]; 
                Jd  = [Jd; Jd_cidx];
            end
        end
        
        function [Jx, Jdx]  = getContactJacobianPar(m, x, mode)
            Jx = []; Jdx = [];
            switch mode
                case 1
                    linkidx = 5; contactLoc = m.mcParam.kneeLoc;
                case 3
                    linkidx = 3; contactLoc = m.mcParam.kneeLoc; % kneeLoc column vec                
                case {2,4}
                    linkidx = []; contactLoc = [];
                case 5
                    linkidx = [3, 5];
                    contactLoc = [m.mcParam.kneeLoc, m.mcParam.kneeLoc];
            end
            
            for cidx = 1:length(linkidx)
                [Jx_cidx, Jdx_cidx] = getJacobianPar(x, linkidx(cidx), contactLoc(:,cidx));
                Jx  = [Jx; Jx_cidx]; 
                Jdx  = [Jdx; Jdx_cidx];
            end
        end
        
        function Initialize_model(m,varargin)
            % do nothing for WB model
        end
    end
end