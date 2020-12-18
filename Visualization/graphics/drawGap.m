function o = drawGap(xmid, w)
v = [xmid-w/2, -8, 0; xmid+w/2, -8, 0; xmid+w/2, 8, 0; xmid-w/2, 8, 0];
f = [1, 2, 3, 4];
c = [0,1,0];

p = patch('faces', f, 'vertices', v, 'FaceVertexCData', c, 'FaceColor', 'flat');
set(p, 'FaceLighting','flat'); % Set the renderer
set(p, 'FaceColor','flat');     % Set the face coloring
set(p,'AmbientStrength',0.6);
o.v = v;
o.p = p;
end