function c_obj = drawCone(r,l,c)
    [v f1 f2 cvec1 cvec2] = createCone(r,l,c);
    
    c_obj.v = v;
    p = patch('faces', f1, 'vertices', v, 'FaceVertexCData',cvec1, 'FaceColor', 'flat');
    set(p, 'FaceLighting','flat'); % Set the renderer
    set(p, 'FaceColor','flat');     % Set the face coloring
    set(p, 'EdgeColor','none');     % Don't show edges  
    set(p, 'AmbientStrength',.5);
    c_obj.p(1) = p;
    
    p = patch('faces', f2, 'vertices', v, 'FaceVertexCData',cvec2, 'FaceColor', 'flat');
    set(p, 'FaceLighting','flat'); % Set the renderer
    set(p, 'FaceColor','flat');     % Set the face coloring
    set(p, 'EdgeColor','none');     % Don't show edges  
    set(p, 'AmbientStrength',.5);
    c_obj.p(2) = p;
return

function [v f1 f2 cvec cvec2] = createCone(r,l,c)
    persistent baseVerts faces1 faces2 slices
    if length(baseVerts) == 0
       slices = 20;
       theta = 0:(2*pi/slices):2*pi;
       theta = theta';
       theta(end) = [];
       baseVerts = [cos(theta) sin(theta) 0*theta ; 0 0 1 ];
       for i = 1:slices
           im = mod(i+1,slices);
           if im == 0;
               im = slices;
           end
           faces1(end+1,:) = [i im slices+1];
       end
       faces2 = [1:slices];
    end
    
    v = baseVerts*diag([r r l]);
    f1=faces1;
    f2=faces2;
    cvec = ones(slices,3)*diag(c);
    cvec2 = ones(1,3)*diag(c);
return
