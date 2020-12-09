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
            if nargin > 1
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
            sim.kneeCloud = [zeros(1, length(-sim.model.knee:space:0));
                        -sim.model.knee:space:0];
        end
    end
    
    methods
        function set_Controller(sim, Controller)
            sim.controller = Controller;
        end
        
        function run_Controller(sim)
            sim.controller.run();
        end
    end
    
    methods % collistion and touchdown detection
        function collision = selfCollision(sim, x)
            collision = 0;
            q = x(1:7,1);            
                                    
            for leg = 1:2
                if leg == 1
                    otherLeg = 2;
                else
                    otherLeg = 1;
                end
                knee_Body = sim.model.getPositionBodyFrame(q,2*leg+1,[0 0]);
                % check knee collision with body
                [dist, ~] = pointToCloud(knee_Body, simbodyCloud);
                if dist < thresh
                    collision = 1;
                    return
                end
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
                % check foot collision with body                              
                foot_Body = sim.model.getPositionBodyFrame(q,2*leg+1,[0, -sim.model.kneeLinkLength]);
                [dist, ~] = pointToCloud(foot_Body, sim.bodyCloud);
                if dist < thresh
                    collision = 1;
                    return
                end
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
                        if ~any(sim.touchDown(x,j,currentMode,collisionList)==1) && j > 30
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
                if any(strcmp(collisionList, "Ffoot")) && j > 30                    
                    TD(1) = 1;
                end
            end            
            % If back foot is in collision and current mode is second
            % flight, fire back foot TD
            if currentMode == 4
                if any(strcmp(collisionList, "Bfoot")) && j > 30                    
                    TD(2) = 1;
                end
            end
        end
        
        function collisionList = groundCollision(sim, x)
            q = x(1:7, 1);
            thresh = 0.001;
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
        function [X, predidx] = run(sim, x0, delay)
            currentMode = sim.scheduledSeq(1);
            % preallocat enough memomery for X to save time
            X = zeros(sim.model.xsize, sim.scheduledHorizons(1)+delay+10);
            X(:,1) = x0;
            k = 1;
            while 1
                xk = X(:, k);
                % selft-collision detection
                if sim.selfCollision(x)
                    return
                end
                % falldown detection
                if sim.fallDectection(x,k,currentMode)
                    return
                end
                % joint limit detection
                if sim.jointLimitViolation(x)
                    return
                end
                % touchdown detection
                TD = sim.touchDown(x,k,currentMode);
                if any(TD)
                    break;
                end            
                
                if k <= sim.controlHorizon
                    uk = sim.run_Controller(xk, k, currentMode);
                else % execture last control command for late contact
                    uk = sim.run_Controller(xk, sim.controlHorizon, currentMode);
                end                
                [xk_next, y] = sim.model.dynamics(xk, uk, currentMode);
                X(:, k+1) = xk_next;
                k = k + 1;
            end
            if any(TD)
                [xk_next,~] = sim.model.resetmap(xk, currentMode);  
                X(:, k+1) = xk_next;
                k = k + 1;
                predidx = k;
            end
            
            % rollout in delay time            
            currentMode = sim.scheduledSeq(2); % shift the current mode            
            for kdelay = 1:delay-1
                xk = X(:,k);
                uk = sim.run_Controller(xk, kdelay, currentMode);
                [xk_next, y] = sim.model.dynamics(xk, uk, currentMode);
                X(:,k+1) = xk_next;
                k = k + 1;
            end
            delayidx = k;
            X(:,delayidx+1:end)=[]; % remove surplus preallocated memory
            X(:,1) = []; % remove the initial condition
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
            if sim.contrlHorizon > phaseHorizons(1)-1
                sim.contrlHorizon = phaseHorizons(1)-1;
                fprintf('control horizon is restrcited to %d \n', sim.contrlHorizon);
            end
        end
    end
end