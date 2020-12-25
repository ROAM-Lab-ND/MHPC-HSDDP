classdef constraint < handle
    properties
        WBModel
        FBModel
    end
    
    methods % constructor
        function cs = constraint(wbmodel, fbmodel)
            if nargin > 0
                cs.WBModel = wbmodel;
                cs.FBModel = fbmodel;
            end
        end
    end
    
    methods
        function ineqInfo = WB_bounding_ineq_constr(cs,x,u,y,mode,flag)           
            C = []; b = [];
            ncs = 0;
            usize = cs.WBModel.usize;
            xsize = cs.WBModel.xsize;
            ysize = cs.WBModel.ysize;
            qsize = cs.WBModel.qsize;
            switch mode
                case {1,3}
                    ncs = 11;
                case {2,4}
                    ncs = 8;
            end
                               
            %% Torque limit Cu*u + bu > = 0
            Cu = [-eye(usize);
                eye(usize)];
            
            bu = 33*ones(2*usize, 1);
            
            % Put them in a big matrix s.t. Cu*u+bu>=0 <-> C*[x;u;y] + b >=0
            C  = [C;
                [zeros(2*usize, xsize), Cu, zeros(2*usize, ysize)]];
            b  = [b;
                bu];
            %% Joint limit Cq*q + bq > = 0
            Cjoint = [-eye(4);
                eye(4)];
            
            bjoint =  [ pi/4;      % front hip min
                -0.1;      % front knee min
                1.15*pi;   % back hip min
                -0.1;      % back knee min
                pi;        % front hip max
                pi-0.2;    % front knee max
                0.1;       % back hip max
                pi-0.2];   % back knee max
            
            C = [C;
                zeros(8,3), Cjoint, zeros(8, qsize), zeros(8,usize), zeros(8, ysize)];
            b = [b;
                bjoint];
            
            %% Friction constraint Cy*y + by >=0
            mu_fric = 0.6; % static friction coefficient
            if mode == 3
                Cy = [0,    1,       0, 0;
                    -1    mu_fric, 0, 0;
                    1     mu_fric, 0, 0];
                by = zeros(3,1);
            elseif mode == 1
                Cy = [0, 0, 0,    1;
                    0, 0, -1,   mu_fric;
                    0, 0, 1,    mu_fric];
                by = zeros(3,1);
            end
            
            if any(mode == [1, 3])
                C = [C;
                    zeros(3, xsize), zeros(3, usize), Cy];
                b = [b;
                    by];
            end
            switch flag
                case 'nopar'
                    ineqInfo = C*[x; u; y] + b;
                case 'par'
                    ineqInfo = ineqInfoStruct(ncs, xsize, usize,ysize);
                    ineqInfo.c = C*[x; u; y] + b;
                    ineqInfo.cx = C(:,1:xsize);
                    ineqInfo.cu = C(:,xsize+1:xsize+usize);
                    ineqInfo.cy = C(:,end-ysize+1:end);
                    ineqInfo.cxx = zeros(xsize, xsize, size(C,1));
                    ineqInfo.cuu = zeros(usize, usize, size(C,1));
                    ineqInfo.cyy = zeros(ysize, ysize, size(C,1));
                otherwise
                    error('Wrong input! flag must be either "nopar" or "par"');
            end
            
        end
        
        function [h,hx,hxx] = WB_bounding_tm_constr(cs,x,mode)
            switch mode
                case 2
                    [h, hx, hxx] = WB_FL1_terminal_constr(x);
                case 4
                    [h, hx, hxx] = WB_FL2_terminal_constr(x);
                otherwise
                    h = 0; hx = zeros(1, cs.WBModel.xsize); hxx = zeros(cs.WBModel.xsize);
            end
        end
        
        function ineqInfo = WB_jumping_ineq_constr(cs,x,u,y,mode,flag)
            ineqInfo = cs.WB_bounding_ineq_constr(x,u,y,mode,flag);
            
            % preallocate memory for gradient and hessian of jumping
            % constraint
            if strcmp(flag, 'par')
                cu = zeros(1, cs.WBModel.usize);
                cy = zeros(1, cs.WBModel.ysize);
                cuu = zeros(cs.WBModel.usize);
                cyy = zeros(cs.WBModel.ysize);
                cxx = zeros(cs.WBModel.xsize);
            end
                            
            link = [3, 5];
            for idx = 1:2
                p = cs.WBModel.getPosition(x, link(idx), [0,-cs.WBModel.kneeLinkLength]');
                [rgap, drgap, ddrgap] = gapFunc(p(1)); % evaluate the quadratic function induced by the gap
                c = p(2) - rgap;
                if strcmp(flag, 'nopar')
                    ineqInfo = [ineqInfo; c];
                else
                    [J,~] = cs.WBModel.getJacobian(x, link(idx), [0,-cs.WBModel.kneeLinkLength]');
                    [Jx,~] = cs.WBModel.getJacobianPar(x, link(idx), [0,-cs.WBModel.kneeLinkLength]');
                    
                    cx = [J(2,:), zeros(1,cs.WBModel.qsize)] - [drgap*J(1,:),zeros(1,cs.WBModel.qsize)];
                    cxx = [squeeze(Jx(2,:,:));zeros(cs.WBModel.qsize, cs.WBModel.xsize)] - ...
                        [squeeze(Jx(1,:,:));zeros(cs.WBModel.qsize, cs.WBModel.xsize)]*drgap -...
                        blkdiag(J(1,:)'*J(1,:),zeros(cs.WBModel.qsize))*ddrgap;
                    
                    % add to ineqInfo
                    ineqInfo.add_constraint(c,cx,cu,cy,cxx,cuu,cyy);
                end                
            end                                    
        end
        
        function ineqInfo = FB_jumping_ineq_constr(cs,x)
            ncs = 2;
            ineqInfo = ineqInfoStruct(ncs,cs.FBModel.xsize, cs.FBModel.usize, cs.FBModel.ysize);
            % Front foothold location of virtually fixed leg
            qWB = [x(1:3)',0.3*pi,-0.7*pi,0.3*pi,-0.7*pi];
            
            cu = zeros(1, cs.FBModel.usize);
            cy = zeros(1, cs.FBModel.ysize);
            cuu = zeros(cs.FBModel.usize);
            cyy = zeros(cs.FBModel.ysize);
                
            link = [3,5];
            for idx = 1:2
                p = cs.WBModel.getPosition(qWB, link(idx), [0, cs.WBModel.kneeLinkLength]');
                xWB = [qWB; zeros(7,1)];
                [rgap, drgap, ddrgap] = gapFunc(p(1));
                c = p(2) - rgap;
                [J_WB,~] = cs.WBModel.getJacobian(xWB, link(idx), [0,-cs.WBModel.kneeLinkLength]');
                [Jx_WB,~] = cs.WBModel.getJacobianPar(xWB, link(idx), [0,-cs.WBModel.kneeLinkLength]');
                
                cx = [J_WB(2,1:3), zeros(1,cs.FBModel.qsize)] - [drgap*J_WB(1,1:3),zeros(1,cs.FBModel.qsize)];
                cxx = blkdiag(Jx_WB(2,1:3,1:3),zeros(cs.FBModel.qsize)) - ...
                    blkdiag(Jx_WB(1,1:3,1:3),zeros(cs.FBModel.qsize))*drgap -...
                    blkdiag(J_WB(1,1:3)'*J(1,1:3),zeros(cs.FBModel.qsize))*ddrgap;                
                
                % add to ineqInfo
                ineqInfo.add_constraint(c,cx,cu,cy,cxx,cuu,cyy);
            end                         
           
        end
    end
    
    
end