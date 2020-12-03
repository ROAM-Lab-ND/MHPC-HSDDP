function updateObject(os,p,R,sz)
    if nargin == 3
        sz = [1 1 1]';
    end
    if isfield(os,'objs')
       os = os.objs; 
    end
    
    for i = 1:length(os)
        o = os(i);
        v = TransformVertices(o.v*diag(sz), R,p);
        for i = 1:length(o.p)
            set(o.p(i),'vertices',v);
            set(o.p(i),'Visible','on');
        end
    end
end