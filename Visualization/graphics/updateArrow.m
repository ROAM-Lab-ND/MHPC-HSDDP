function updateArrow(o,p,f)
    nf = norm(f);
    
    if nf > 1e-6
        fn = f / nf;
        v  = [0 0 1];
        ax = cross(v)*fn;
        ax = ax/norm(ax);
        ang = acos(dot(v,fn));
        R   = expm(cross(ang*ax));

        updateObject(o,p,R,[1 1 nf]);
    else
        updateObject(o,p,eye(3),[1 1 nf]);
    end
end
    
