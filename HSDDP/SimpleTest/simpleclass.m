classdef simpleclass
    properties
        a = 1;
        b = 1;
        c = 0;
    end
    
    properties
        add
    end
    
    methods
        function obj = simpleclass()
        end
         
        function output(obj)
            c = obj.add(obj.a, obj.b);
            obj.c = 1 + c;
            fprintf('c = %d, obj.c = %d', c, obj.c);
        end
    end
end