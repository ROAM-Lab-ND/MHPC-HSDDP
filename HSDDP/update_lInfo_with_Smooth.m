function lInfo = update_lInfo_with_Smooth(lInfo,u,upre,dt,eps_smooth)
S = eye(length(u));
lInfo.l = lInfo.l + 0.5 * eps_smooth *(u-upre)'*S*(u-upre)*dt;
lInfo.lu = lInfo.lu + eps_smooth*S*(u-upre)*dt;
lInfo.luu = lInfo.luu + eps_smooth*S*dt;
end