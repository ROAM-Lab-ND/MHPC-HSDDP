function phiInfo = terminal_cost_info(x,xd,Q)
phiInfo.phi = 1/2*(x-xd)'*Q*(x-xd);
phiInfo.phix = Q*(x-xd);
phiInfo.phixx = Q;
end