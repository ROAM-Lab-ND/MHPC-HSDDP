function [f,df,ddf] = gapFunc(x)
x0 = 0.96;
w = 0.5;
h = 0.07;
f = (-2*sqrt(h)/w)*((x-x0).^2 - w^2/4) - 0.404;
df = (-4*h^(0.5)/w)*(x-x0);
ddf = -4*h^(0.5)/w;

% %%%%%%% nth order approximation of rectangular function %%%%%
% n = 8;
% f = 1./((2/w*(x-x0)).^n + 1) - 1 + h - 0.404;
% df = -(f.^2*2*n/w) .* (2/w*(x-x0)).^(n-1);
% ddf = -(4*n*f.*df/w) .* (2/w*(x-x0)).^(n-1) - 4*n*(n-1)*f.^2/(w^2) .* (2/w*(x-x0)).^(n-2);
end