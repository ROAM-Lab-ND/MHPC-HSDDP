
X0 = eye(2);
z=[]
dt = .001;
for t = 0:dt:1
    b = t*(1-t);
    dX = [ 0 1; b 0]*X0;
    dX*dt
    pause(1)
    X0 = X0 + dX*dt;
end
X0
