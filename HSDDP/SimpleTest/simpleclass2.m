classdef simpleclass2
    
    methods
        function obj = simpleclass2()
        end
         
%         function c = add(obj, a, b)
%             c = a + b;
%         end
        function c = add(obj, a, b)
            c = add(a,b);
        end
    end
end