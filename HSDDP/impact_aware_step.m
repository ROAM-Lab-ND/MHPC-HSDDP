function [G_new,H_new] = impact_aware_step(phiInfo, Px, G_prime,H_prime)
    G_new       = phiInfo.phix + Px*G_prime;
    H_new       = phiInfo.phixx + H_prime*Pxx*H_prime;
end