function [cost, G, H] = AugmentedLagrangian(h, hx, hxx, sigma, lambda)
% h equality constraint in column vec
% hx, hxx gradient and hessian of equality constraints
cost = (sigma/2)^2*(h'*h) + lambda'*h;
xsize = size(hx, 1);
G = zeros(xsize, 1);
H = zeros(xsize, xsize);

% Gradient
G = (sigma/2)^2*2*hx*h + hx*lambda;

% Hessian
for i = 1:length(h)
    hi = h(i);
    hix = hx(:,i);
    hixx = hxx(:,:,i);
    lambdai = lambda(i);
    H = H + (sigma/2)^2*2*((hix*hix') + hi*hixx) + lambdai*hixx;
end
end