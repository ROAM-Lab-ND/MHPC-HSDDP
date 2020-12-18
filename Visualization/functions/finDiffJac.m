function [ Jac ] = finDiffJac( func, x_0, diff_eps)
%finDiffJac evaluates a finite difference jacobian matrix
%   [ Jac ] = finDiffJac( func, x_0, diff_eps)
%   func:     function to use for eval
%   x_0:      operating point
%   diff_eps: forward difference perturbation

f_0 = func(x_0);
Jac = zeros(length(f_0), length(x_0));
for i = 1:length(x_0)
  x_i = x_0;
  x_i(i) = x_i(i) + diff_eps;
  Jac(:,i) = (func(x_i) - f_0) / diff_eps;
end

end

