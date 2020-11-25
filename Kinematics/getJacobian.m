function [J,Jd] = getJacobian(x,linkidx,contactLoc)
% This function computes Jacobian and Jacobian Derivative for contact point
% on link linkidx with local position contactLoc
% This function would be implemented with Spatial Vector method in future
% release
switch linkidx
    case 1
        [J,Jd] = Link1Jacobian(x, contactLoc);
    case 2
        [J,Jd] = Link2Jacobian(x, contactLoc);
    case 3
        [J,Jd] = Link3Jacobian(x, contactLoc);
    case 4
        [J,Jd] = Link4Jacobian(x, contactLoc);
    case 5
        [J,Jd] = Link5Jacobian(x, contactLoc);
end
end