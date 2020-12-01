function lInfo = running_cost_info(x,xd,u,ud,y,yd,Q,R,S,dt)
lInfo.l     = 2*dt*1/2*((x-xd)'*Q*(x-xd) + (u-ud)'*R*(u-ud) + (y-yd)'*S*(y-yd));
lInfo.lx    = 2*dt*Q*(x-xd); % column vec
lInfo.lu    = 2*dt*R*(u-ud);
lInfo.ly    = 2*dt*S*(y-yd);
lInfo.lxx   = 2*dt*Q;
lInfo.lux   = zeros(length(u),length(x));
lInfo.luu   = 2*dt*R;
lInfo.lyy   = 2*dt*S;
end