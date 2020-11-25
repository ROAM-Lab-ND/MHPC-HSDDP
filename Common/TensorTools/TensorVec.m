function M = TensorVec(Ax, b)
% This function computes the product of tensor Ax and vector b.
% Essentially, this function computes the first term of of partial of
% (A(x)b(x)) w.r.t. x.
% Ax is the jacobian of A(x) w.r.t. x. Suppose x has dim n, b has dim m
% If A(x) has dim 1 x m, then Ax has dim m x n
% If A(x) has dim q x m, then Ax has dim q x m x n

dim = size(Ax);
if length(dim) <= 2
    M = b'*Ax;
else
    M = zeros(dim(1),dim(3));
    for row = 1:dim(1)
        M(row, :) = b'*squeeze(Ax(row,:,:));
    end
end
end