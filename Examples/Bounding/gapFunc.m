function [f,df,ddf] = gapFunc(x)
x0 = 1.5;
w = 0.6;
h = 0.05;
% f = (-2*sqrt(h)/w)*((x-x0).^2 - w^2/4) - 0.404;
% df = (-4*h^(0.5)/w)*(x-x0);
% ddf = -4*h^(0.5)/w;

% %%%%%%% nth order approximation of rectangular function %%%%%
n = 6;
s = 0.4;
f = s*(1./((2/w*(x-x0)).^n + 1) - 1)  + h - 0.404;
df = -(2*n*s*((2*(x - x0))/w).^(n - 1))./(w*(((2*x - 2*x0)/w).^n + 1).^2);
ddf = (8*n^2*s*((2*(x - x0))/w).^(2*n - 2))./(w^2*(((2*x - 2*x0)/w).^n + 1).^3) - (4*n*s*(n - 1)*((2*(x - x0))/w).^(n - 2))/(w^2*(((2*x - 2*x0)/w).^n + 1).^2);
end