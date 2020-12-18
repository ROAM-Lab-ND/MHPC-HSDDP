function testDraw()
f = figure(198)
clf
%f = subplot(211)
hold on
% set(f,'Name','3D-Output of a bounding quadruped');  % Window title
set(f,'Color','w');         % Background colo
set(f,'Renderer','OpenGL')

drawFloor()

quad.body = drawBox([.5 .3 .2],[.5 .3 .2]/2,[1 1 1]);
for i = 1:4
    quad.hip(i) = drawCylinder(.05, .5,[0 0 1]);
    quad.shank(i) = drawCylinder(.05, .5,[0 0 1]);
    quad.force(i) = drawArrow(.1 , .02 ,[1 0 1]);
end
    
    
camproj('perspective');
camtarget([0, 0, +0.6])
campos ([5, -10, +2.6]);
camup ([0,0,1]);
camva(15)
axis off
box off
axis equal
light('Position',[5 -10 12 ],'Style','local','Color',[.7 .7 .7]);
drawnow();




tic
kk = 0;
for x0 = 0:.1:2*pi
    R0 = expm(cross([0 0 x0]));
    R1 = expm(cross([0 x0 0]));
    R2 = expm(cross([2*x0 x0 0]));
    
    dp = [cos(x0) sin(x0) 0]';
    
    kk = kk+1;
   
    
    updateObject(quad.body,dp*0+[0 0 .5]',R0);
    
    for i = 1:4
        fi = [.2 .3 .7]';
        
        updateObject(quad.hip(i),dp+[1 i 0]',R0);
        updateObject(quad.shank(i),dp+[0 i 0]',R1);
        updateArrow(quad.force(i),dp+[i 0 0]',fi);
    end     
    
    camtarget([0, 0, +0.6]+dp')
    campos ([5, -10, +2.6]+dp');
    drawnow();

end
toc
kk/toc

end
