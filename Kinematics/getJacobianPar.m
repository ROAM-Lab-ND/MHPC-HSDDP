function [Jx, Jdx] = getJacobianPar(x,linkidx,contactLoc)
switch linkidx
    case 1
        [Jx,Jdx] = Link1Jacobian_par(x, contactLoc);
    case 2
        [Jx,Jdx] = Link2Jacobian_par(x, contactLoc);
    case 3
        [Jx,Jdx] = Link3Jacobian_par(x, contactLoc);
    case 4
        [Jx,Jdx] = Link4Jacobian_par(x, contactLoc);
    case 5
        [Jx,Jdx] = Link5Jacobian_par(x, contactLoc);
end
end