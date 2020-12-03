function updateObject(os)
    if isfield(os,'objs')
       os = os.objs; 
    end
    
    for i = 1:length(os)
        o = os(i);
        for i = 1:length(o.p)
            set(o.p(i),'Visible','off');
        end
    end
end