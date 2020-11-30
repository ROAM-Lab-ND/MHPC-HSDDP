function build2DminiCheetahFB(mcFB)
mc3DParams = get3DMCParams();
mc2DParams = get2DMCParams(mc3DParams);
[Inertia, mass] = recomputeInertia(mc2DParams);
mcFB.Inertia = Inertia;
mcFB.mass    = mass;
end