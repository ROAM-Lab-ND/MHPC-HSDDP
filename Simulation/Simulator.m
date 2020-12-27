classdef Simulator < handle
    properties
        groundInfo
        model   % dynamics model of planar Quadruped
        
        controller
        scheduledSeq    % scheduled phase sequence
        scheduledHorizons
        controlHorizon
    end
    
    properties
        bodyCloud
        kneeCloud
        hipCloud
    end
    
    methods % constructor
        function sim = Simulator(model)
            if nargin > 0
                Initialization(sim, model);
            end
        end
        
        function Initialization(sim, model)
            sim.model = model;
            
            % body Contour in body frame
            space = 0.005;
            leftTop = [-sim.model.bodyLength/2, sim.model.bodyHeight/2]';
            rightBottom = [sim.model.bodyLength/2, -sim.model.bodyHeight/2]';
            sim.bodyCloud = getRectangle(leftTop, rightBottom, space)';
            
            % hip and knee cloud in local frame
            sim.hipCloud = [zeros(1, length(-sim.model.hipLinkLength:space:0));
                        -sim.model.hipLinkLength:space:0];
            sim.kneeCloud = [zeros(1, length(-sim.model.kneeLinkLength:space:0));
                        -sim.model.kneeLinkLength:space:0];
        end
    end
    
    methods
        function set_Controller(sim, Controller)
            sim.controller = Controller;
        end
        
        function u = run_Controller(sim, x, k, currentPhaseIdx)
            u = sim.controller.run(x, k, currentPhaseIdx);
        end
    end
    
    methods % collistion and touchdown detection
        function collision = selfCollision(sim, x)
            collision = 0;
            q = x(1:7,1);            
            thresh = 0.003;                        
            for leg = 1:2
                if leg == 1
                    otherLeg = 2;
                else
                    otherLeg = 1;
                end
                knee_Body = sim.model.getPositionBodyFrame(q,2*leg+1,[0 0]');
                
                % check knee collision with hip link of the other leg
                [dist, ~] = pointToCloud(knee_Body, sim.model.getPositionBodyFrame(q, 2*otherLeg, sim.hipCloud));
                if dist < thresh
                    collision = 1;
                    return
                end
                % check knee collision with knee link of the other leg
                [dist, ~] = pointToCloud(knee_Body, sim.model.getPositionBodyFrame(q, 2*otherLeg+1, sim.kneeCloud));
                if dist < thresh
                    collision = 1;
                    return
                end
%                 % check foot collision with body                              
                foot_Body = sim.model.getPositionBodyFrame(q,2*leg+1,[0, -sim.model.kneeLinkLength]');
%                 [dist, ~] = pointToCloud(foot_Body, sim.bodyCloud);
%                 if dist < thresh
%                     collision = 1;
%                     return
%                 end
                % check foot collision with hip link of the other leg
                [dist, ~] = pointToCloud(foot_Body, sim.model.getPositionBodyFrame(q, 2*otherLeg, sim.hipCloud));
                if dist < thresh
                    collision = 1;
                    return
                end
                % check foot collision with knee link of the other leg
                [dist, ~] = pointToCloud(foot_Body, sim.model.getPositionBodyFrame(q, 2*otherLeg+1, sim.kneeCloud));
                if dist < thresh
                    collision = 1;
                    return
                end
            end
        end
        
        function violation = jointLimitViolation(sim, x)
            violation = 0;
        end
        
        function fall = fallDectection(sim, x, j, currentMode)
            % j time step in current mode
            q = x(1:7, 1);
            fall = 0;
            collisionList = sim.groundCollision(x);
            % If body and any knee is in collision, it is a fallwon
            if any(strcmp(collisionList, "Body"))
                fall = 1;
                return
            end            
            if any(strcmp(collisionList, "Fknee"))
                fall = 1;
                return;
            end            
            if any(strcmp(collisionList, "Bknee"))
                fall = 1;
                return;
            end
            
            switch currentMode
                case {2,4}
                    if any(strcmp(collisionList, "Ffoot")) || any(strcmp(collisionList, "Bfoot"))
                        if ~any(sim.touchDown(x,j,currentMode,collisionList)==1) && j > 25
                            fall = 1;
                            return
                        end
                    end
                case 1 % If front foot in collision during bs, fall
                    if any(strcmp(collisionList, "Ffoot"))
                        fall = 1;
                        return
                    end
                case 3 % If back foot in collision during fs, fall
                    if any(strcmp(collisionList, "Bfoot"))
                        fall = 1;
                        return
                    end
            end
        end 
        
        function TD = touchDown(sim, x, j, currentMode, collisionList)
            TD = [0 0];
            if nargin < 5
                collisionList = sim.groundCollision(x);
            end
            % If front foot is in collision and current mode is first
            % flight, fire front foot TD
            if currentMode == 2
                if any(strcmp(collisionList, "Ffoot"))                    
                    TD(1) = 1;
                end
            end            
            % If back foot is in collision and current mode is second
            % flight, fire back foot TD
            if currentMode == 4
                if any(strcmp(collisionList, "Bfoot"))                    
                    TD(2) = 1;
                end
            end
        end
        
        function collisionList = groundCollision(sim, x)
            q = x(1:7, 1);
            thresh = 0.003;
            collisionList = [];
            bodyLength = sim.model.bodyLength;
            bodyHeight = sim.model.bodyHeight;
            % corner points of body in body frame
            bodyCheckPoints = [-bodyLength/2*ones(1,2), bodyLength/2*ones(1,2);
                               -bodyHeight/2,bodyHeight/2,bodyHeight/2,-bodyHeight/2];
            candidate = sim.model.getPosition(q,1,bodyCheckPoints);
            % check body collision
            if(min(candidate(2,:)-sim.groundInfo)<thresh)
                collisionList = [collisionList, "Body"];
                return
            end
            
            % check front knee collision
            candidate = sim.model.getPosition(q,3,[0 0]');
            if(min(candidate(2,:)-sim.groundInfo)<thresh)
                collisionList = [collisionList, "Fknee"];
                return
            end
            
            % check back knee collision
            candidate = sim.model.getPosition(q,5,[0 0]');
            if(min(candidate(2,:)-sim.groundInfo)<thresh)
                collisionList = [collisionList, "Bknee"];
                return
            end
            
            % check front foot collision
            candidate = sim.model.getPosition(q,3,[0 -sim.model.kneeLinkLength]');
            if(min(candidate(2,:)-sim.groundInfo)<thresh)
                collisionList = [collisionList, "Ffoot"];                
            end            
            
            % check back foot collision
            candidate = sim.model.getPosition(q,5,[0 -sim.model.kneeLinkLength]');
            if(min(candidate(2,:)-sim.groundInfo)<thresh)
                collisionList = [collisionList, "Bfoot"];                
            end 
        end        
    end
    
    methods
        function [X, U, Y, t, predidx, collision] = run(sim, x0, t0, delay, disturbInfo)
            currentMode = sim.scheduledSeq(1);
            nextMode = sim.scheduledSeq(2);
            pidx = 1;
            predidx = 1;    % index of predicted initial condition of next phase
            push = [];
            pushLoc = [];
            pushlinkidx = [];
            collision = 0;
            % preallocat enough memomery for X,U,Y to save time
            X = zeros(sim.model.xsize, sim.scheduledHorizons(1)+delay+10+1);
            U = zeros(sim.model.xsize, sim.scheduledHorizons(1)+delay+10);
            Y = zeros(sim.model.xsize, sim.scheduledHorizons(1)+delay+10);
            t = zeros(1, sim.scheduledHorizons(1)+delay+10);
            
            X(:,1) = x0;
            k = 1;
            tk = t0;
            while 1
                xk = X(:, k);
%                 selft-collision detection
                if sim.selfCollision(xk)
                    collision = 1;                   
                end
                % falldown detection
                if sim.fallDectection(xk,k,currentMode)
                    collision = 1; 
                end
% %                 joint limit detection
%                 if sim.jointLimitViolation(xk)
%                     collision = 1; 
%                 end

                if collision == 1
                    % remove surplus preallocated memory
                    U(:, k:end) = [];
                    X(:, k+1:end)=[]; 
                    Y(:, k:end) = [];
                    t(:, k:end) = [];
                    return;
                end
                
                % touchdown detection during flight phase
                if any(currentMode == [2 4])
                    TD = sim.touchDown(xk,k,currentMode);
                    % If touchdown, break
                    if any(TD)
                        break;
                    end
                else
                    % If maximum control horizon is reached (during stance), break
                    if k == sim.controlHorizon + 1
                        break;
                    end
                end                        
                
                if k <= sim.controlHorizon
                    uk = sim.run_Controller(xk, k, pidx);
                % execture last control command for late contact
                else 
                    uk = sim.run_Controller(xk, sim.controlHorizon, pidx);
                end             
                
                if disturbInfo.active && (k >= disturbInfo.start) && (k<=disturbInfo.end)
                    pushlinkidx = 1;
                    pushLoc = datasample(sim.bodyCloud,1,2); % random sample from body cloud
                    push = disturbInfo.magnitude*[cos(randn);sin(randn)];
                end
                % execute continuous dynamics
                [xk_next, yk] = sim.model.dynamics(xk, uk, currentMode, push, pushLoc, pushlinkidx);
                U(:, k) = uk;
                Y(:, k) = yk;
                X(:, k+1) = xk_next;
                t(:, k) = tk;
                k = k + 1;
                tk = tk + sim.model.dt;
                predidx = k;
            end
            % execute resetmap if either touchdown happens or maximum
            % control horizon is reached
            [xk_next,~] = sim.model.resetmap(xk, currentMode, nextMode);
            X(:, k+1) = xk_next;
            t(:, k) = tk;
            k = k + 1;
            predidx = k;
            
            % rollout in delay time            
            currentMode = nextMode; % shift the current mode   
            pidx = 2;
            for kdelay = 1:delay-1
                xk = X(:,k);
                uk = sim.run_Controller(xk, kdelay, pidx);
                [xk_next, yk] = sim.model.dynamics(xk, uk, currentMode);
                U(:, k) = uk;
                Y(:, k) = yk;              
                X(:,k+1) = xk_next;
                t(:, k) = tk;
                k = k + 1;
                tk = tk + sim.model.dt;
            end
            
            % remove surplus preallocated memory
            U(:, k:end) = [];
            X(:, k+1:end)=[]; 
            Y(:, k:end) = [];
            t(:, k:end) = [];
        end
    end
    
    methods % setup and update functions
        function set_groundInfo(sim, groundInfo)
            sim.groundInfo = groundInfo;
        end
        
        function set_horizonParams(sim, phaseSeq, phaseHorizons, contrlHorizon)
            % Restrict the control horizon to not exceed the horizon of
            % first phase in scheduled phase sequence
            if contrlHorizon > phaseHorizons(1)-1
                contrlHorizon = phaseHorizons(1)-1;
                fprintf('control horizon is restrcited to %d \n', contrlHorizon);
            end
           
            sim.scheduledSeq = phaseSeq;
            sim.scheduledHorizons = phaseHorizons;
            sim.controlHorizon = contrlHorizon;
            
        end
        
        function recalcHorizonforDelay(sim, lastDelay)
            % re-calculate scheduced horizon to account for delay of last
            % phase
            sim.scheduledHorizons(1) = sim.scheduledHorizons(1)-lastDelay;
            if sim.controlHorizon > sim.scheduledHorizons(1)-1
                sim.controlHorizon = sim.scheduledHorizons(1)-1;
                fprintf('control horizon is restrcited to %d \n', sim.controlHorizon);
            end
        end
    end
end