classdef PlanarFloatingBase < BaseDyn
    properties (SetAccess=private, GetAccess=public)
        dt
        qsize = 3; 
        xsize = 6;        
        usize = 4; 
        ysize = 4;       
        p = zeros(4,1); % foothold location
        s = ones(2, 1); % contact state
        WBModel
    end
    
    properties % inertia params
        Inertia
        mass
    end
    
    methods
        % constructor
        function m = PlanarFloatingBase(dt) 
            m.dt = dt;  
            m.WBModel = PlanarQuadruped(dt);
            build2DminiCheetah(m.WBModel);
            recomputeInertia(m);
        end
    end
    
    methods                
        function [x_next, y] = dynamics(m,x,u,mode)
            switch mode
                case 1
                    m.s = [0, 1]';
                case 3
                    m.s = [1, 0]';  
                case {2, 4}
                    m.s = [0, 0]';                              
                case 5
                    m.s = [1, 1]';
            end
            fcon    = FBDynamics(x,u,m.p,m.s);
            x_next  = x + fcon * m.dt;
            y       = u;
        end
        
        function [x_next, y] = resetmap(m,x,mode,next_mode)
            x_next = x;
            y = zeros(m.ysize, 1);
        end
    end
    
    methods
        function dynInfo = dynamics_par(m,x,u,mode)
            switch mode
                case 1
                    m.s = [0, 1]';
                case {2, 4}
                    m.s = [0, 0]';
                case 3
                    m.s = [1, 0]';               
                case 5
                    m.s = [1, 1]';
            end
            [fconx,fconu] = FBDynamics_par(x,u,m.p,m.s);
            dynInfo.fx = eye(m.xsize) + fconx*m.dt;
            dynInfo.fu = fconu*m.dt;
            dynInfo.gx = zeros(m.ysize, m.xsize);
            dynInfo.gu = eye(m.usize);
        end
        
        function  Px = resetmap_par(m,x,mode,next_mode)
            Px = eye(m.xsize);
            gx = zeros(m.ysize, m.xsize);
        end
    end
    
    methods
        function Initialize_model(m, p)           
            set_footholdLoc(m, p);
        end
        
        function set_footholdLoc(m, p)
            m.p = p;
        end
        
        function set_contactState(m, cState)
            m.s = cState;
        end
    end
    
    methods
        function recomputeInertia(m)
            ihat = [1 0 0]';
            
            wbQuad = m.WBModel;
            m.mass = wbQuad.robotMass;
            
            dist_CoM_hip = dot(wbQuad.hipLoc{1} + (ry(-pi/2))'*wbQuad.hipLinkCoM, ihat);
            dist_CoM_knee = dist_CoM_hip + norm(wbQuad.kneeLinkCoM);
            
            eqv_hipRotInertia =  wbQuad.hipRotInertia + wbQuad.hipLinkMass*dist_CoM_hip^2*0.85;
            eqv_kneeRotInertia = wbQuad.kneeRotInertia + wbQuad.kneeLinkMass*dist_CoM_knee^2*0.6;
            
            eqv_bodyRotInertia = wbQuad.bodyRotInertia + 2*eqv_hipRotInertia + 2*eqv_kneeRotInertia;
%             [~, D] = eig(eqv_bodyRotInertia);
            
            m.Inertia = eqv_bodyRotInertia(2,2);
        end
    end
end