function lInfo = update_lInfo_with_ReB(lInfo, ineq_Info, delta, eps)
% Copy ineq constraint infrom from ineq_Info
c  = ineq_Info.c; 
cx = ineq_Info.cx; 
cu = ineq_Info.cu; 
cy = ineq_Info.cy;


[B, Bc, Bcc]   = ReducedBarrier(c, delta); % Compute ReB and its partials for ineq

lInfo.l = lInfo.l + eps*B; % Update running cost (same eps for all ineqs)

xsize = length(lInfo.lx);
usize = length(lInfo.lu);
ysize = length(lInfo.ly);
csize = length(c);

% Preallocate memory
Bx     = zeros(xsize,csize);
Bu     = zeros(usize,csize);
By     = zeros(ysize,csize);
Bxx    = zeros(xsize,xsize,csize);
Buu    = zeros(usize,usize,csize);
Byy    = zeros(ysize,ysize,csize);

% Compute Reduced barrier info w.r.t x, u and y
for cidx = 1:csize
    Bx(:,cidx)      =  Bc(cidx)*cx(cidx,:)';
    Bu(:,cidx)      =  Bc(cidx)*cu(cidx,:)';
    By(:,cidx)      =  Bc(cidx)*cy(cidx,:)';
    Bxx(:,:,cidx)   =  cx(cidx,:)'*Bcc(cidx)*cx(cidx,:);
    Buu(:,:,cidx)   =  cu(cidx,:)'*Bcc(cidx)*cu(cidx,:);
    Byy(:,:,cidx)   =  cy(cidx,:)'*Bcc(cidx)*cy(cidx,:);
end

% Update lInfo
% Assumes the same weighting param eps is used for all ineqs
% eps row vector of weighting params
lInfo.lx = lInfo.lx + Bx*eps';
lInfo.lu = lInfo.lu + Bu*eps';
lInfo.ly = lInfo.ly + By*eps';

Buu_wsum = zeros(usize, usize);
Bxx_wsum = zeros(xsize, xsize);
Byy_wsum = zeros(ysize, ysize);

for cidx = 1:csize
    Buu_wsum = Buu_wsum + eps(cidx)*Buu(:,:,cidx);
    Bxx_wsum = Bxx_wsum + eps(cidx)*Bxx(:,:,cidx);
    Byy_wsum = Byy_wsum + eps(cidx)*Byy(:,:,cidx);
end
lInfo.luu = lInfo.luu + Buu_wsum;
lInfo.lxx = lInfo.lxx + Bxx_wsum;
lInfo.lyy = lInfo.lyy + Byy_wsum;
end