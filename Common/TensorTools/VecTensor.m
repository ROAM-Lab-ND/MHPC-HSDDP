function VecTensor(y, fxx)
% f and y are column vector of dim m, and x is column vector of dim n
% This function computes second partial of y'f w.r.t. x assuming y fxied
% fxx stacks f1xx, f2xx, ... fmxx in the third dimension
dim = size(fxx);
M = zeros(dim(1),dim(2));
for elem = 1:dim(1)
    M = M + y(elem)*squeeze(fxx(:,:,elem));
end
end