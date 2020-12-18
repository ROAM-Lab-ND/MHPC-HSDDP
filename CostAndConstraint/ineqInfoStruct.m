classdef ineqInfoStruct < handle
    properties
        c
        cx
        cu
        cy
        cxx
        cuu
        cyy
    end
    methods
        function cs = ineqInfoStruct(neq, xsize, usize, ysize)
            if nargin > 0
                cs.c = zeros(neq, 1);
                cs.cx = zeros(neq, xsize);
                cs.cu = zeros(neq, usize);
                cs.cy = zeros(neq, ysize);
                cs.cxx = zeros(neq, xsize, xsize);
                cs.cuu = zeros(neq, usize, usize);
                cs.cyy = zeros(neq, ysize, ysize);
            end
        end
        
        function add_constraint(cs,c,cx,cu,cy,cxx,cuu,cyy)
            cs.c = [cs.c; c];
            cs.cx = [cs.cx; cx];
            cs.cu = [cs.cu; cu];
            cs.cy = [cs.cy; cy];
            cs.cxx = cat(3,cs.cxx, cxx);
            cs.cuu = cat(3,cs.cuu, cuu);
            cs.cyy = cat(3,cs.cyy, cyy);
        end
    end
end