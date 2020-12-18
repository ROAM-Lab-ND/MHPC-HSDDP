classdef lInfoStruct
    properties
        l
        lx
        lu
        ly
        lxx
        lux
        luu
        lyy
    end
    
    methods 
        function lInfo = lInfoStruct(xsize,usize,ysize)
            if nargin > 0
                lInfo.l = 0;
                lInfo.lx = zeros(xsize, 1);
                lInfo.lu = zeros(usize, 1);
                lInfo.ly = zeros(ysize, 1);
                lInfo.lxx = zeros(xsize);
                lInfo.lux = zeros(usize, xsize);
                lInfo.luu = zeros(usize);
                lInfo.lyy = zeros(ysize);
            end
        end
        
        function lInfo = add_lInfo(lInfo, lInfoToAdd)
            lInfo.l = lInfo.l + lInfoToAdd.l;
            lInfo.lx = lInfo.lx + lInfoToAdd.lx;
            lInfo.lu = lInfo.lu + lInfoToAdd.lu;
            lInfo.ly = lInfo.ly + lInfoToAdd.ly;
            lInfo.lxx = lInfo.lxx + lInfoToAdd.lxx;
            lInfo.lux = lInfo.lux + lInfoToAdd.lux;
            lInfo.luu = lInfo.luu + lInfoToAdd.luu;
            lInfo.lyy = lInfo.lyy + lInfoToAdd.lyy;
        end
    end
end