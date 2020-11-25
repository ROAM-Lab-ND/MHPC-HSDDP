function lInfo = running_cost_info(x,xd,u,ud,y,yd,Q,R,S,dt)
lInfo.l     = dt*1/2*((x-xd)'*Q*(x-xd) + (u-ud)'*R*(u-ud) + (y-yd)'*S*(y-yd));
lInfo.lx    = dt*Q*(x-xd); % column vec
lInfo.lu    = dt*R*(u-ud);
lInfo.ly    = dt*S*(y-yd);
lInfo.lxx   = dt*Q;
lInfo.lux   = zeros(length(u),length(x));
lInfo.luu   = dt*R;
lInfo.lyy   = dt*S;
end