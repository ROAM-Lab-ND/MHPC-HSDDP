function [h,hx,hxx] = WB_terminal_constr_Info(x, mode)
switch mode
    case 2
        [h, hx, hxx] = WB_FL1_terminal_constr(x);
    case 4
        [h, hx, hxx] = WB_FL2_terminal_constr(x);
    otherwise
        xsize = length(x);
        h = 0; hx = zeros(1, xsize); hxx = zeros(xsize, xsize);
end
end