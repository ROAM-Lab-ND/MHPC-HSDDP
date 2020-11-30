classdef PlanarFloatingBase < BaseDyn
    properties (SetAccess=protected, GetAccess=public)
        dt
        qsize = 3; 
        xsize = 6;        
        usize = 4; 
        ysize = 4;       
        p = zeros(4,1); % foothold location
        s = ones(2, 1); % contact state
    end
    
    properties % inertia params
        Inertia
        mass
    end
    
    methods
        % constructor
        function m = PlanarFloatingBase(dt)           
            m.dt = dt;                       
        end
    end
    
    methods                
        function [x_next, y] = dynamics(m,x,u,mode)
            switch mode
                case 1
                    m.s = [0, 1];
                case 3
                    m.s = [1, 0];  
                case {2, 4}
                    m.s = [0, 0]';                              
                case 5
                    m.s = [1, 1];
            end
            fcon    = FBDybamics(x,u,m.p,m.s);
            x_next  = x + fcon * m.dt;
            y       = u;
        end
        
        function [x_next, y] = resetmap(m,x,mode,next_mode)
            x_next = x;
            y = zeros(m.ysize, 1);
        end
    end
    
    methods
        function [fx, fu, gx, gu] = dynamics_par(m,x,u,mode)
            switch mode
                case 1
                    m.s = [0, 1];
                case {2, 4}
                    m.s = [0, 0]';
                case 3
                    m.s = [1, 0];               
                case 5
                    m.s = [1, 1];
            end
            [fconx,fconu] = FBDynamics_par(x,u,m.p,m.s);
            fx = eye(m.xsize) + fconx*m.dt;
            fu = fconu*m.dt;
            gx = zeros(m.ysize, m.xsize);
            gu = eye(m.usize);
        end
        
        function  Px = resetmap_par(m,x,mode,next_mode)
            Px = eye(m.xsize);
            gx = zeros(m.ysize, m.xsize);
        end
    end
    
    methods
        function Initialize_model(m, x0, vd, T)
            px = x0(1,1) + vd*T;
            pz = -0.404;
            footLoc = [px, pz]';
            set_footholdLoc(m, footLoc);
        end
        
        function set_footholdLoc(m, p)
            m.p = p;
        end
        
        function set_contactState(m, cState)
            m.s = cState;
        end
    end
end