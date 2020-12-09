classdef mhpcController < RobotController
    properties
        xopt  % optimal state given by HSDDP
        uopt  % optimal feedforward by HSDDP
        Kopt  % optimal feedback gain by HSDDP
        delay
    end
    
    methods % constructor
        function C = mhpcController(xopt, uopt, Kopt)
            if nargin > 0
                Initialization(C, xopt, uopt, Kopt);
            end
        end
        
        function updateController(C, xopt, uopt, Kopt)
            Initialization(C, xopt, uopt, Kopt);
        end
        
        function Initialization(C, xopt, uopt, Kopt)
            C.xopt = xopt;
            C.uopt = uopt;
            C.Kopt = Kopt;
        end
        
        function InformControllerDelay(C, delay)
            C.delay = delay;
            C.xopt{1}(:,1:delay) = [];
            C.uopt{1}(:,1:delay) = [];
            C.Kopt{1}(:,:,1:delay) = [];
        end
        
        function uk = run(C,xk,k,pidx)
            uk = C.uopt{pidx}(:,k) + C.Kopt{pidx}(:,:,k)*(xk - C.xopt{pidx}(:,k));
        end
    end
    
end