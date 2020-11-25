function [B, Bz, Bzz] = ReducedBarrier(z, delta)
% ReB Reduced barrier function penalizing z > = 0
% delta(i) relaxation parameter
% Bz, Bzz Derivative of B w.r.t. z
assert(length(z)<=length(delta), 'ineq size and relaxation parameter size do not match.');
k = 2;      % second order polynominal extension
B = zeros(length(z),1);
Bz = zeros(length(z), 1);
Bzz = zeros(length(z), 1);
for i = 1:length(z)
    if z(i)>delta(i)
        B(i) = -log(z(i));
        Bz(i) = -z(i)^(-1);
        Bzz(i) = z(i)^(-2);
    else
        B(i) = (k-1)/k*(((z(i)-k*delta(i))/((k-1)*delta(i)))^k - 1) - log(delta(i));
        Bz(i) =  ((z(i)-k*delta(i))/((k-1)*delta(i)))^(k-1)/delta(i);
        Bzz(i) = ((z(i)-k*delta(i))/((k-1)*delta(i)))^(k-2);
    end
end

end
