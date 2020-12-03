function os = drawArrow(perc,r,c)
    o = drawCylinder(r,(1-perc),c);
    os.objs(1) = o;
    
    o = drawCone(r*2,perc,c);
    o.v = TransformVertices(o.v,eye(3),[0 0 1-perc]);
    os.objs(2) = o;
    updateObject(os.objs(2), [0 0 0]',eye(3));
end
    
