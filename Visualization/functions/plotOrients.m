
figure(6)
clf
axis([-1 1 -1 1 -1 1])
hold on

axs = eye(3);
zs = zeros(3);

pts0 = [zs(:,1) axs(:,1) zs(:,2) axs(:,2) zs(:,3) axs(:,3)];
pts = pts0;

h = plot3(pts(1,:), pts(2,:), pts(3,:),'r');

figure(7)
clf
hold on
plot(Time, omega(:,1) ,'r') ;
plot(Time, omega(:,2) ,'g') ;
plot(Time, omega(:,3) ,'b') ;
%rpy = 0*quat;
%rpy = rpy(:,1:3);

figure(8)
clf
hold on
plot(Time,rpy(:,1),'r');
plot(Time,rpy(:,2),'g');
plot(Time,rpy(:,3),'b');



dYaw = [0 ;diff(rpy(:,3))./diff(Time)];
pdYaw = 0;
for i = 2:length(dYaw)
   if dYaw(i) == 0
       dYaw(i) = pdYaw;
   else
       pdYaw = dYaw(i);
   end
end

figure(9)
clf;
hold on;
%plot(Time,dYaw/3,'b');
plot(Time,omega(:,3)-dYaw/3,'g');

%plot(Time,omega0(:,3),'r');

%omega0 = 0*omega;
%return

figure(10);
clf;
subplot(311); hold on;
plot(Time,rpy(:,1),'r')
plot(Time,rpyMad(:,1),'b')

subplot(312); hold on;
plot(Time,rpy(:,2),'r')
plot(Time,rpyMad(:,2),'b')

subplot(313); hold on;
plot(Time,rpy(:,3),'r')
plot(Time,rpyMad(:,3),'b')



%rpyMad = 0*rpy;

step = 3000/10;
step = 1;
for i = 1:step:length(Time)
    %rpy(i,:) = quatToRpy(quat(i,:))';
    
    rpyMad(i,:) = quatern2euler( quat(i,:) );
    
    
     R = quatToR(quat(i,:)');
    % omega0(i,:) = ( R*omega(i,:)' )' ;
     
%     pts = R*pts0;
%     set(h,'XData',pts(1,:),'YData',pts(2,:),'ZData',pts(3,:));
%     
%    pause(1/20.); 
end
