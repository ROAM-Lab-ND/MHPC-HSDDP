classdef Graphics < handle
    properties
        model
        trajectory
    end
    methods
        function G = Graphics(robotParams)
            G.model.g       = [0 0 -9.81]';
            G.model.p_hip   = robotParams.abadLoc; % ignores abad length
            G.model.l1      = robotParams.hipLinkLength;
            G.model.l2      = robotParams.kneeLinkLength;
        end
    end
    
    methods
        function visualize(G, options)            
            bigAnim = figure(198);
%             set(bigAnim, 'Position', get(0, 'Screensize'));
            clf
            %f = subplot(211)
            hold on
            % set(f,'Name','3D-Output of a bounding Quadruped');  % Window title
            set(bigAnim,'Color','w');         % Background colo
            set(bigAnim,'Renderer','OpenGL')
            drawFloor()
            
            l = G.model.p_hip{1}(1) - G.model.p_hip{4}(1);
            w = -G.model.p_hip{1}(2) + G.model.p_hip{4}(2);
            h = w/3;
            
            % Initialize objects
            Quad.body = drawBox([l w h],-G.model.p_hip{3}',[225,124,22]/256*1.2);
            
%             Quad.push = drawArrow(.1 , .01 ,[1 0 0]);  % initialize  arrow
            
            for i = 1:4
                Quad.hip(i) = drawCylinder(.015, G.model.l1,[92 51 23]/256*1.5);
                Quad.shank(i) = drawCylinder(.015, G.model.l2,[92 51 23]/256*1.5);
            end
                        
            camproj('perspective');
            camtarget([-0.4, -0.4, +0.2])
            view([65 10])
            camup ([0,0,1]);
            camva(1.3)
            axis off
            box off
            axis equal
            light('Position',[5 20 12 ],'Style','local','Color',[.7 .7 .7]);
            drawnow();
            
            updateObject(Quad.body,[0 0 1]',eye(3));
%             updateArrow(Quad.push,[0 0 0]',[0 0 0]');
                        
            Rbase = expm(cross([0 pi/2 0]));
                        
            x_c           = G.trajectory.x_c; % continuously updating part of state
            x_d           = G.trajectory.x_d; % discretely updating part of state
            q             = G.trajectory.q;
%             push         = trajectory.push;
%             push_loc     = trajectory.push_loc;
            
            size_x_c = size(x_c);
            t        = G.trajectory.t;
            
            M = struct('cdata',[],'colormap',[]);
            for n = 1:size_x_c(2)
                p       = x_c(4:6,n);
                rpy     = x_c(1:3,n);
                R       = rpyToR(rpy);
%                 f       = push(:,n);
%                 f_loc   = push_loc(:,n);
                
                if p(3) > 0.7    % hide object if flows above this height
                    hideObject(Quad.body);
                    for i = 1:4
                        hideObject(Quad.hip(i));
                        hideObject(Quad.shank(i));
                    end
                else
                    updateObject(Quad.body,p, R);
%                     updateArrow(Quad.push, f_loc, f/500);
                    
                    for i = 1:4
                        if x_d.leg_state{i} == 0
                            hideObject(Quad.hip(i));
                            hideObject(Quad.shank(i));
                        else                            
                            qn = q(3*(i-1)+1:3*i,n);
                            [~, Rshank, pKnee,RHip] = G.legKinematics(qn);
                            
                            updateObject(Quad.hip(i), p+R*G.model.p_hip{i},R*RHip*Rbase);
                            updateObject(Quad.shank(i), p+R*G.model.p_hip{i}+R*pKnee,R*Rshank*Rbase);
                            %                 updateArrow(Quad.force(i),p+R*(model.p_hip{i}+pFoot),f_s{i}/500);
                        end
                    end
                end
%                 if norm(f) == 0
%                     hideObject(Quad.push);
%                 end
                figure(198)
                camtarget([p(1),p(2), .2]);
                drawnow;
                Frame = getframe;
                M(end+1) = Frame;               
            end
            M = M(2:end);
            v = VideoWriter('Multiple_Quad.avi');
            open(v)
            for i = 1:length(M)
                writeVideo(v, M(i));
            end
            close(v);
        end
        
        function [p, R, p1, R1] = legKinematics(G, q)
            %SimPeriod Simulate one full period
            t1 = q(1);
            t2 = q(2);
            t3 = q(3);
            l1 = G.model.l1;
            l2 = G.model.l2;
            
            
            R01 = expm(cross([t1 0 0]));
            R12 = expm(cross([0 t2 0]));
            R23 = expm(cross([0 t3 0]));
            p1 = R01*R12 * [ l1 ; 0 ;0];
            R1 = R01*R12;
            
            p = p1 + R01*R12*R23*[ l2 ; 0 ;0];
            R = R01*R12*R23;
        end
        
        function process2DData(G, X)
            x_c_2D = X(1:3,:);
            q_2D   = X(4:7,:);
            step = 4;
            dt = 0.001*step;
            N = length(X(1,1:step:end));
            t = dt*(0:1:N-1);
            
            x_c_2D = x_c_2D(:,1:step:end);
            q_2D = q_2D(:,1:step:end);
            
            pos = [x_c_2D(1,:); zeros(1,N); x_c_2D(2,:)+0.404];   % pos
            rpy = [zeros(1,N); x_c_2D(3,:); zeros(1,N)];    % Euler angle
            qjoint = [zeros(1,N);pi/2+q_2D(1,:);q_2D(2,:); zeros(1,N);pi/2+q_2D(1,:);q_2D(2,:);  % zero hip aligns body in this case
                zeros(1,N);pi/2+q_2D(3,:);q_2D(4,:); zeros(1,N);pi/2+q_2D(3,:);q_2D(4,:)]; % joint angle
            
            G.trajectory.x_c = [rpy; pos];
            G.trajectory.x_d.leg_state = {1,1,1,1}';
            G.trajectory.t = t;
            G.trajectory.q = qjoint;
            G.trajectory.control_params = struct([]);  % place holder
        end
    end
end