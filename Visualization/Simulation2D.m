function Simulation2D(x, model, F_ext, F_extLocal)
q = x(1:model.qsize,:);
len_sim = size(x, 2);
if nargin <=5
    F_ext = zeros(2, len_sim);
    F_extLocal = zeros(2, len_sim);
end
F_ext = F_ext/100;
xmid = 0.55;
w = 0.3;
%% link data w.r.t. local frame
bodyLength = model.bodyLength+0.04;
bodyHeight = model.bodyHeight+0.04;
hipLength = model.hipLinkLength;
kneeLength = model.kneeLinkLength;
linkwidth = 0.03;

bodyLocal = [-bodyLength/2, -bodyLength/2,  bodyLength/2,  bodyLength/2; 
             bodyHeight/2,  -bodyHeight/2,  -bodyHeight/2,  bodyHeight/2];

hipLocal = [-linkwidth/2,    -linkwidth/2,  linkwidth/2,  linkwidth/2;
           0,               -hipLength,    -hipLength,   0];
       
kneeLocal = [-linkwidth/2,    -linkwidth/2,  linkwidth/2,  linkwidth/2;
           0,               -kneeLength,    -kneeLength,   0];
       
q0 = q(:,1);

bodyWorld = model.getPosition(q0, 1, bodyLocal);
FhipWorld =  model.getPosition(q0, 2, hipLocal);
FkneeWorld = model.getPosition(q0, 3, kneeLocal);
BhipWorld =  model.getPosition(q0, 4, hipLocal);
BkneeWorld = model.getPosition(q0, 5, kneeLocal);

%% Animition intilization
f=figure;
h(1)=patch(bodyWorld(1,:),bodyWorld(2,:),'red');
hold on;
h(2)=patch(FhipWorld(1,:),FhipWorld(2,:),'blue');
h(3)=patch(FkneeWorld(1,:),FkneeWorld(2,:),'green');
h(4)=patch(BhipWorld(1,:),BhipWorld(2,:),'blue');
h(5)=patch(BkneeWorld(1,:),BkneeWorld(2,:),'green');
plot([-1 xmid-w/2; xmid+w/2 4]',-0.404*ones(2,2),'k-','linewidth',1.5);
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
    t = model.dt*(k-1);
    if b > 0.01
        qk = q(:,k);
        bodyWorld = model.getPosition(qk, 1, bodyLocal);
        FhipWorld =  model.getPosition(qk, 2, hipLocal);
        FkneeWorld = model.getPosition(qk, 3, kneeLocal);
        BhipWorld =  model.getPosition(qk, 4, hipLocal);
        BkneeWorld = model.getPosition(qk, 5, kneeLocal);
        
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
    pause(model.dt);    
end

filename = 'Bounding2D.gif';           
if ~isempty(F_ext)
    for k=1:length(im)
        [imind,cm] = rgb2ind(im{k},256);
        if k == 1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',model.dt);
        elseif k==length(im)
            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',2);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',model.dt);
        end
    end
end

end