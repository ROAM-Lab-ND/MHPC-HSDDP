function phiInfo = update_terminalInfo_with_AL(h, hx, hxx, phiInfo, sigma, lambda)

% h equality constraint in column vec
% hx, hxx Jacobin (partial x in colum dir) and hessian of equality constraints
cost = (sigma/2)^2*(h'*h) + lambda'*h;
xsize = size(hx, 1);

G = zeros(xsize, 1);
H = zeros(xsize, xsize);

% Gradient of AL term
G = (sigma/2)^2*2*(h*hx)' + (lambda*hx)';

% Hessian of AL term
for i = 1:length(h)
    hi = h(i);
    hix = hx(:,i);
    hixx = hxx(:,:,i);
    H = H + (sigma/2)^2*2*((hix'*hix) + hi*hixx) + lambda(i)*hixx;
end

% Update terminal cost
phiInfo.phi     = phiInfo.phi   + 50*cost;          
phiInfo.phix    = phiInfo.phix  + 50*G;
phiInfo.phixx   = phiInfo.phixx + 50*H;
end