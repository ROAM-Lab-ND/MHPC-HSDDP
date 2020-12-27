classdef Graphics < handle
    properties
        Gmodel
        Gmodel2D
        trajectory3D
        trajectory2D
        model        
    end
    methods
        function G = Graphics(quad3D, quad2D)
            G.Gmodel.g       = [0 0 -9.81]';
            G.Gmodel.p_hip   = quad3D.abadLoc; % ignores abad length
            G.Gmodel.l1      = quad3D.hipLinkLength;
            G.Gmodel.l2      = quad3D.kneeLinkLength;
            G.model = quad2D;
            
             %% link data w.r.t. local frame
            bodyLength = quad2D.bodyLength+0.04;
            bodyHeight = quad2D.bodyHeight+0.04;
            hipLength = quad2D.hipLinkLength;
            kneeLength = quad2D.kneeLinkLength;
            linkwidth = 0.03;
            
            G.Gmodel2D.bodyLocal = [-bodyLength/2, -bodyLength/2,  bodyLength/2,  bodyLength/2;
                            bodyHeight/2,  -bodyHeight/2,  -bodyHeight/2,  bodyHeight/2];
            
            G.Gmodel2D.hipLocal = [-linkwidth/2,    -linkwidth/2,  linkwidth/2,  linkwidth/2;
                                0,               -hipLength,    -hipLength,   0];
            
            G.Gmodel2D.kneeLocal = [-linkwidth/2,    -linkwidth/2,  linkwidth/2,  linkwidth/2;
                                0,               -kneeLength,    -kneeLength,   0];
            
        end
    end
    
    methods
        function setTrajectory(G, simTrajectory)
            for pidx = 1:length(simTrajectory)
                [pitch, pos2D, q2D] = G.get2DConfig(simTrajectory(pidx).X);
                G.trajectory2D(pidx).x_c = [pos2D;pitch];
                G.trajectory2D(pidx).q   = q2D;
                G.trajectory2D(pidx).t = simTrajectory(pidx).t;
                G.trajectory2D(pidx).U = simTrajectory(pidx).U;
                G.trajectory2D(pidx).Y = simTrajectory(pidx).Y;
                
                [pitch_d, pos2D_d, q2D_d] = G.get2DConfig(simTrajectory(pidx).Xopt);
                G.trajectory2D(pidx).x_c_d = [pitch_d; pos2D_d];
                G.trajectory2D(pidx).q_d   = q2D_d;
                
                [rpy, pos, q] = G.get3DConfig(simTrajectory(pidx).X);
                G.trajectory3D(pidx).x_c = [rpy; pos];
                G.trajectory3D(pidx).x_d.leg_state = {1,1,1,1}';
                G.trajectory3D(pidx).t = simTrajectory(pidx).t;
                G.trajectory3D(pidx).q = q;
                G.trajectory3D(pidx).control_params = struct([]);  % place holder
                
                [rpy_d, pos_d, q_d] = G.get3DConfig(simTrajectory(pidx).Xopt);
                G.trajectory3D(pidx).x_c_d = [rpy_d; pos_d];
                G.trajectory3D(pidx).x_d_d.leg_state = {1,1,1,1}';
                G.trajectory3D(pidx).q_d = q_d;
            end
        end
    end
    
    methods
        function visualize(G, options)
            if strcmp(options.view, '2D')
                visualize2D(G, options);
            elseif strcmp(options.view, '3D')
                visualize3D(G, options);
            else
                error('view option has to be either 2D or 3D');
            end            
        end
        
        function visualize3D(G, options)      
            
            bigAnim = figure(198);
%             set(bigAnim, 'Position', get(0, 'Screensize'));
            clf
            hold on
            % set(f,'Name','3D-Output of a bounding Quadruped');  % Window title
            set(bigAnim,'Color','w');         % Background colo
            set(bigAnim,'Renderer','OpenGL')
            drawFloor()
            drawGap(options.gapLoc, options.gapWidth);
            ln_plan = plot3(0,0,0,'r--','linewidth',1.5);
            
            l = G.Gmodel.p_hip{1}(1) - G.Gmodel.p_hip{4}(1);
            w = -G.Gmodel.p_hip{1}(2) + G.Gmodel.p_hip{4}(2);
            h = w/3;
            
            % Initialize objects
            Quad.body = drawBox([l w h],-G.Gmodel.p_hip{3}',[225,124,22]/256*1.2);
            for i = 1:4
                Quad.hip(i) = drawCylinder(.015, G.Gmodel.l1,[92 51 23]/256*1.5);
                Quad.shank(i) = drawCylinder(.015, G.Gmodel.l2,[92 51 23]/256*1.5);
            end
                        
            % Set camera and light
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
            
            Rbase = expm(Cross([0 pi/2 0]));                                   
            
            M = struct('cdata',[],'colormap',[]);
            
            step = 10;            
            for pidx = 1:length(G.trajectory3D)
                x_c           = G.trajectory3D(pidx).x_c(:,1:step:end); % continuously updating part of state
                x_d           = G.trajectory3D(pidx).x_d(:,1:step:end); % discretely updating part of state
                q             = G.trajectory3D(pidx).q(:,1:step:end);        
                t        = G.trajectory3D(pidx).t(:,1:step:end);
                size_x_c = size(x_c);
                
                % Update planned trajectory
                if options.showPlan
                    set(ln_plan, 'XData', x_c(4,:), 'YData', x_c(5,:), 'ZData',x_c(6,:));
                end
                
                % Update actual configuration
                for n = 1:size_x_c(2)
                    p       = x_c(4:6,n);
                    rpy     = x_c(1:3,n);
                    R       = rpyToR(rpy);
                               
                    if p(3) > 0.7    % hide object if flows above this height
                        hideObject(Quad.body);
                        for i = 1:4
                            hideObject(Quad.hip(i));
                            hideObject(Quad.shank(i));
                        end
                    else
                        updateObject(Quad.body,p, R);                        
                        for i = 1:4
                            if x_d.leg_state{i} == 0
                                hideObject(Quad.hip(i));
                                hideObject(Quad.shank(i));
                            else
                                qn = q(3*(i-1)+1:3*i,n);
                                [~, Rshank, pKnee,RHip] = G.legKinematics(qn);
                                
                                updateObject(Quad.hip(i), p+R*G.Gmodel.p_hip{i},R*RHip*Rbase);
                                updateObject(Quad.shank(i), p+R*G.Gmodel.p_hip{i}+R*pKnee,R*Rshank*Rbase);
                            end
                        end
                    end
                    
                    figure(198)
                    camtarget([p(1),p(2), .2]);
                    drawnow;
                    Frame = getframe;
                    M(end+1) = Frame;
                end
            end
           
            filename = [options.filename, '3D.avi'];
            M = M(2:end);
            v = VideoWriter(filename);
            open(v)
            for i = 1:length(M)
                writeVideo(v, M(i));
            end
            close(v);
        end
        
        function visualize2D(G, options)
            
            F_ext = [];
            F_extLocal = [];
            F_ext = F_ext/100;
                      
            C0 = [G.trajectory2D(1).x_c(:,1); G.trajectory2D(1).q(:,1)]; % Initial Configuration
            
            bodyWorld = G.model.getPosition(C0, 1, G.Gmodel2D.bodyLocal);
            FhipWorld =  G.model.getPosition(C0, 2, G.Gmodel2D.hipLocal);
            FkneeWorld = G.model.getPosition(C0, 3, G.Gmodel2D.kneeLocal);
            BhipWorld =  G.model.getPosition(C0, 4, G.Gmodel2D.hipLocal);
            BkneeWorld = G.model.getPosition(C0, 5, G.Gmodel2D.kneeLocal);
            
            %% Animition intilization
            f=figure;
            h(1)=patch(bodyWorld(1,:),bodyWorld(2,:),'red');
            hold on;
            h(2)=patch(FhipWorld(1,:),FhipWorld(2,:),'blue');
            h(3)=patch(FkneeWorld(1,:),FkneeWorld(2,:),'green');
            h(4)=patch(BhipWorld(1,:),BhipWorld(2,:),'blue');
            h(5)=patch(BkneeWorld(1,:),BkneeWorld(2,:),'green');
            ln_plan = plot(0,0,'m--','linewidth', 1.5);
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
            
            
            %% Update animation
            a = tic;
            im_idx = 1;
            
            for pidx = 1:length(G.trajectory2D)
                len_phase = length(G.trajectory2D(pidx).t);
                
                % Update planned trajectory
                if options.showPlan
                    set(ln_plan, 'XData', G.trajectory2D(pidx).x_c(1,:), ...
                             'YData', G.trajectory2D(pidx).x_c(2,:));   
                end                             
                
                % Update actual configuration 
                for k=1:len_phase
                    b = toc(a);        
                    tk = G.trajectory2D(pidx).t(k);
                    if b > 0.01
                        Ck = [G.trajectory2D(pidx).x_c(:,k); G.trajectory2D(pidx).q(:,k)];
                        bodyWorld = G.model.getPosition(Ck, 1, G.Gmodel2D.bodyLocal);
                        FhipWorld =  G.model.getPosition(Ck, 2, G.Gmodel2D.hipLocal);
                        FkneeWorld = G.model.getPosition(Ck, 3, G.Gmodel2D.kneeLocal);
                        BhipWorld =  G.model.getPosition(Ck, 4, G.Gmodel2D.hipLocal);
                        BkneeWorld = G.model.getPosition(Ck, 5, G.Gmodel2D.kneeLocal);
                        
                        h(1).XData = bodyWorld(1,:); h(1).YData = bodyWorld(2,:);
                        h(2).XData = FhipWorld(1,:); h(2).YData = FhipWorld(2,:);
                        h(3).XData = FkneeWorld(1,:); h(3).YData = FkneeWorld(2,:);
                        h(4).XData = BhipWorld(1,:); h(4).YData = BhipWorld(2,:);
                        h(5).XData = BkneeWorld(1,:); h(5).YData = BkneeWorld(2,:);
                        
                        if ~isempty(F_ext)
                            qv.XData = F_extLocal(1,k);   % update GRF plot
                            qv.YData = F_extLocal(2,k);
                            qv.UData = F_ext(1,k);
                            qv.VData = F_ext(2,k);
                        end                        
                        
                        th.String = sprintf('t = %0.3f s', tk);
                        drawnow limitrate nocallbacks;
                        
                        frame{im_idx} = getframe(f);
                        im{im_idx} = frame2im(frame{im_idx});
                        im_idx = im_idx + 1;
                        a = tic;
                    end
                    pause(G.model.dt);
                end
            end            
            
            filename = [options.filename, '2D.gif'];
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
    end
    
    methods
        function plot(G, input)
            t = [G.trajectory2D.t];
            x = [G.trajectory2D.x_c(1,:)];
            z = [G.trajectory2D.x_c(2,:)];
            pitch = [G.trajectory2D.x_c(3,:)];
            q = [G.trajectory2D.q];
%             torque = [G.trajectory2D.U];
%             GRF = [G.trajectory2D.Y];
            
            figure;
            switch input
                case 'torque'
                    plot(t, torque, 'linewidth', 1.5);
                    xlabel('time', 'interpreter','latex');
                    ylabel('torque ($N\cdot m$)', 'interpreter','latex');
                    hold on
                    plot(t, -34*ones(1,length(t)), t, 34*ones(1,length(t)), '--','linewidth', 1.0);
                    l = legend('$J_1$','$J_2$','$J_3$','$J_4$', 'max', 'min');
                    l.Interpreter = 'latex';
                case 'pos'
                    subplot(211)
                    plot(t, x, 'linewidth', 1.5);
                    ylabel('x (mm)', 'interpreter','latex');
                    set(gca, 'Fontsize', 12);
                    subplot(212)
                    plot(t, z, 'linewidth', 1.5);
                    ylabel('z (mm)', 'interpreter','latex');
                    xlabel('time', 'interpreter','latex');
                case 'pitch'
                    plot(t, pitch, 'linewidth', 1.5);
                    xlabel('time', 'interpreter','latex');
                    ylabel('pitch (rad)', 'interpreter','latex');
                case 'joint'
                    plot(t, q, 'linewidth', 1.5);
                    xlabel('time', 'interpreter','latex');
                    ylabel('joint angle (rad)', 'interpreter','latex');
                    l = legend('$J_1$','$J_2$','$J_3$','$J_4$');
                    l.Interpreter = 'latex';
            end
            set(gca, 'Fontsize', 12);
        end
    end
    
    methods (Static)
        function [pitch,pos,qjoint] = get2DConfig(X)
            pitch = X(3,:);
            pos   = X(1:2, :);
            qjoint = X(4:7, :);
        end
        
        function [rpy, pos, qjoint] = get3DConfig(X)
            x_c_2D = X(1:3,:);
            q_2D   = X(4:7,:);          
            N = length(X(1,:));
            
            x_c_2D = x_c_2D(:,:);
            q_2D = q_2D(:,:);
            
            pos = [x_c_2D(1,:); zeros(1,N); x_c_2D(2,:)+0.404];   % pos
            rpy = [zeros(1,N); x_c_2D(3,:); zeros(1,N)];    % Euler angle
            qjoint = [zeros(1,N);pi/2+q_2D(1,:);q_2D(2,:); zeros(1,N);pi/2+q_2D(1,:);q_2D(2,:);  % zero hip aligns body in this case
                zeros(1,N);pi/2+q_2D(3,:);q_2D(4,:); zeros(1,N);pi/2+q_2D(3,:);q_2D(4,:)]; % joint angle                
        end
    end
end