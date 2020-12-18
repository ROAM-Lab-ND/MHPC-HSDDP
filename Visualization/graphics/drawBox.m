function o = drawBox(sz,mid,c)
    
    v = [0 1 1 0 0 1 1 0; 1 1 0 0 1 1 0 0; 0 0 0 0 1 1 1 1]'*diag(sz) - [mid ; mid; mid; mid; mid;mid;mid;mid];
    f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7 ; 4 1 5 8; 2 3 7 6];
    c = [c; c; c; c; c; c];

    p = patch('faces', f, 'vertices', v, 'FaceVertexCData', c, 'FaceColor', 'flat');
    set(p, 'FaceLighting','flat'); % Set the renderer
    set(p, 'FaceColor','flat');     % Set the face coloring
    set(p, 'EdgeColor',[.1 .1 .1]);     % Don't show edges
    set(p,'AmbientStrength',.6);
    o.v = v;
    o.p = p;
end

