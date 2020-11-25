function [inertia, robotMass] = recomputeInertia(PlanarQuad)
% This function computes the equivalent mass of the trunk model
% and computes the maximum equivalent Rotational inertia in Cartisen frame

robotMass = PlanarQuad.bodyMass + 2*PlanarQuad.kneeLinkMass + 2*PlanarQuad.hipLinkMass;

dist_CoM_hip = norm(PlanarQuad.hipLoc{1} + ry(-pi/2)*PlanarQuad.hipLinkCoM);
dist_CoM_knee = dist_CoM_hip + norm(PlanarQuad.kneeLinkCoM);

eqv_hipRotInertia =  PlanarQuad.hipRotInertia + PlanarQuad.hipLinkMass*dist_CoM_hip^2*0.85;
eqv_kneeRotInertia = PlanarQuad.kneeRotInertia + PlanarQuad.kneeLinkMass*dist_CoM_knee^2*0.6;

eqv_bodyRotInertia = PlanarQuad.bodyRotInertia + 2*eqv_hipRotInertia + 2*eqv_kneeRotInertia;
[~, D] = eig(eqv_bodyRotInertia);

inertia = D(2,2);

end