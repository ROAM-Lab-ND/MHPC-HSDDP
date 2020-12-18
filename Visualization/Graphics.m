classdef Graphics < handle
    properties
        Gmodel
        trajectory
        model
    end
    methods
        function G = Graphics(robotParams, quadruped)
            G.Gmodel.g       = [0 0 -9.81]';
            G.Gmodel.p_hip   = robotParams.abadLoc; % ignores abad length
            G.Gmodel.l1      = robotParams.hipLinkLength;
            G.Gmodel.l2      = robotParams.kneeLinkLength;
            G.model = quadruped;
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
            drawGap(options.gapLoc, options.gapWidth);
            
            l = G.Gmodel.p_hip{1}(1) - G.Gmodel.p_hip{4}(1);
            w = -G.Gmodel.p_hip{1}(2) + G.Gmodel.p_hip{4}(2);
            h = w/3;
            
            % Initialize objects
            Quad.body = drawBox([l w h],-G.Gmodel.p_hip{3}',[225,124,22]/256*1.2);
            
%             Quad.push = drawArrow(.1 , .01 ,[1 0 0]);  % initialize  arrow
            
            for i = 1:4
                Quad.hip(i) = drawCylinder(.015, G.Gmodel.l1,[92 51 23]/256*1.5);
                Quad.shank(i) = drawCylinder(.015, G.Gmodel.l2,[92 51 23]/256*1.5);
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
                        
            Rbase = expm(Cross([0 pi/2 0]));
                        
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
                            
                            updateObject(Quad.hip(i), p+R*G.Gmodel.p_hip{i},R*RHip*Rbase);
                            updateObject(Quad.shank(i), p+R*G.Gmodel.p_hip{i}+R*pKnee,R*Rshank*Rbase);
                            %                 updateArrow(Quad.force(i),p+R*(Gmodel.p_hip{i}+pFoot),f_s{i}/500);
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
            v = VideoWriter('QuqadrupedBounding.avi');
            open(v)
            for i = 1:length(M)
                writeVideo(v, M(i));
            end
            close(v);
        end
        
        function visualize2D(G, X, options)
            len_sim = size(X, 2);
            if nargin <=5
                F_ext = zeros(2, len_sim);
                F_extLocal = zeros(2, len_sim);
            end
            F_ext = F_ext/100;
           
            %% link data w.r.t. local frame
            bodyLength = G.model.bodyLength+0.04;
            bodyHeight = G.model.bodyHeight+0.04;
            hipLength = G.model.hipLinkLength;
            kneeLength = G.model.kneeLinkLength;
            linkwidth = 0.03;
            
            bodyLocal = [-bodyLength/2, -bodyLength/2,  bodyLength/2,  bodyLength/2;
                bodyHeight/2,  -bodyHeight/2,  -bodyHeight/2,  bodyHeight/2];
            
            hipLocal = [-linkwidth/2,    -linkwidth/2,  linkwidth/2,  linkwidth/2;
                0,               -hipLength,    -hipLength,   0];
            
            kneeLocal = [-linkwidth/2,    -linkwidth/2,  linkwidth/2,  linkwidth/2;
                0,               -kneeLength,    -kneeLength,   0];
            
            q0 = X(1:G.model.qsize,1);
            
            bodyWorld = G.model.getPosition(q0, 1, bodyLocal);
            FhipWorld =  G.model.getPosition(q0, 2, hipLocal);
            FkneeWorld = G.model.getPosition(q0, 3, kneeLocal);
            BhipWorld =  G.model.getPosition(q0, 4, hipLocal);
            BkneeWorld = G.model.getPosition(q0, 5, kneeLocal);
            
            %% Animition intilization
            f=figure;
            h(1)=patch(bodyWorld(1,:),bodyWorld(2,:),'red');
            hold on;
            h(2)=patch(FhipWorld(1,:),FhipWorld(2,:),'blue');
            h(3)=patch(FkneeWorld(1,:),FkneeWorld(2,:),'green');
            h(4)=patch(BhipWorld(1,:),BhipWorld(2,:),'blue');
            h(5)=patch(BkneeWorld(1,:),BkneeWorld(2,:),'green');
            plot([-1 options.gapLoc-options.gapWidth/2; options.gapLoc+options.gapWidth/2 4]', ...
                        -0.404*ones(2,2),'k-','linewidth',1.5);
            plot([-0:0.1:2],gapFunc(0:0.1:2),'k--');
            ylim([-1 3]);
            
            if ~isempty(F_ext)
                qv = quiver(F_extLocal(1,1),F_extLocal(2,1),F_ext(1,1),F_ext(2,1),'linewidth',3);
                qv.Color = 'magenta';
                qv.MaxHeadSize = 1;
            end
            th = text(0,0.5,'t = 0 s');
            axis([-1 4 -1 1]);
            axis equal
            axis manual
            
            
            %% animation
            a = tic;
            t = 0;
            im_ind = 1;
            for k=1:len_sim
                b = toc(a);
                t = G.model.dt*(k-1);
                if b > 0.01
                    qk = X(1:G.model.qsize,k);
                    bodyWorld = G.model.getPosition(qk, 1, bodyLocal);
                    FhipWorld =  G.model.getPosition(qk, 2, hipLocal);
                    FkneeWorld = G.model.getPosition(qk, 3, kneeLocal);
                    BhipWorld =  G.model.getPosition(qk, 4, hipLocal);
                    BkneeWorld = G.model.getPosition(qk, 5, kneeLocal);
                    
                    h(1).XData = bodyWorld(1,:); h(1).YData = bodyWorld(2,:);
                    h(2).XData = FhipWorld(1,:); h(2).YData = FhipWorld(2,:);
                    h(3).XData = FkneeWorld(1,:); h(3).YData = FkneeWorld(2,:);
                    h(4).XData = BhipWorld(1,:); h(4).YData = BhipWorld(2,:);
                    h(5).XData = BkneeWorld(1,:); h(5).YData = BkneeWorld(2,:);
                    
                    qv.XData = F_extLocal(1,k);   % update GRF plot
                    qv.YData = F_extLocal(2,k);
                    qv.UData = F_ext(1,k);
                    qv.VData = F_ext(2,k);
                    
                    th.String = sprintf('t = %0.3f s', t);
                    drawnow limitrate nocallbacks;
                    
                    frame{im_ind} = getframe(f);
                    im{im_ind} = frame2im(frame{im_ind});
                    im_ind = im_ind + 1;
                    a = tic;
                end
                pause(G.model.dt);
            end
            
            filename = 'Bounding2D.gif';
            if ~isempty(F_ext)
                for k=1:length(im)
                    [imind,cm] = rgb2ind(im{k},256);
                    if k == 1
                        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',G.model.dt);
                    elseif k==length(im)
                        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',2);
                    else
                        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',G.model.dt);
                    end
                end
            end
        end
        
        function [p, R, p1, R1] = legKinematics(G, q)
            %SimPeriod Simulate one full period
            t1 = q(1);
            t2 = q(2);
            t3 = q(3);
            l1 = G.Gmodel.l1;
            l2 = G.Gmodel.l2;
            
            
            R01 = expm(Cross([t1 0 0]));
            R12 = expm(Cross([0 t2 0]));
            R23 = expm(Cross([0 t3 0]));
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