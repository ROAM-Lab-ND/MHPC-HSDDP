function ineq_Info = Jump_ineq_constr_Info(x,u,y,mode,model)
qsize = 7;
xsize = 14;
ysize = 4;
usize = 4;

q = x(1:qsize,1);
C = []; b = [];
ineq_Info.c = [];
ineq_Info.cx = [];
ineq_Info.cu = [];
ineq_Info.cy = [];

%% Torque limit Cu*u + bu > = 0
Cu = [-eye(usize);
    eye(usize)];

bu = 33*ones(2*usize, 1);

% Put them in a big matrix s.t. Cu*u+bu>=0 <-> C*[x;u;y] + b >=0
C  = [C;
    [zeros(2*usize, xsize), Cu, zeros(2*usize, ysize)]];
b  = [b;
    bu];
%% Joint limit Cq*q + bq > = 0
Cjoint = [-eye(4);
    eye(4)];

bjoint =  [ pi/4;      % front hip min
    -0.1;      % front knee min
    1.15*pi;   % back hip min
    -0.1;      % back knee min
    pi;        % front hip max
    pi-0.2;    % front knee max
    0.1;       % back hip max
    pi-0.2];   % back knee max

C = [C;
    zeros(8,3), Cjoint, zeros(8, qsize), zeros(8,usize), zeros(8, ysize)];
b = [b;
    bjoint];
 
%% Friction constraint Cy*y + by >=0
mu_fric = 0.6; % static friction coefficient
if mode == 3
    Cy = [0,    1,       0, 0;
        -1    mu_fric, 0, 0;
        1     mu_fric, 0, 0];
    by = zeros(3,1);
elseif mode == 1
    Cy = [0, 0, 0,    1;
        0, 0, -1,   mu_fric;
        0, 0, 1,    mu_fric];
    by = zeros(3,1);
end

if any(mode == [1, 3])
    C = [C;
        zeros(3, xsize), zeros(3, usize), Cy];
    b = [b;
        by];
end

ineq_Info.c = C*[x; u; y] + b;
ineq_Info.cx = C(:,1:xsize);
ineq_Info.cu = C(:,xsize+1:xsize+usize);
ineq_Info.cy = C(:,end-ysize+1:end);

%% Jumping gap inequality constraint
pF = model.getPosition(q, 3, [0,-model.KneeLinkLength]');
JF = model.getJacobian(q, 3, [0,-model.KneeLinkLength]');
[rgap, drgap] = gapFunc(x(1)); % evaluate the quadratic function induced by the gap
cF = pF(2) - rgap;
cFx = [JF(2,:), zeros(1,qsize)] - [drgap,zeros(1,xsize-1)];

pB = model.getPosition(q, 5, [0,-model.KneeLinkLength]');
JB = model.getJacobian(q, 5, [0,-model.KneeLinkLength]');
cB = pB(2) - rgap;
cBx = [JB(2,:), zeros(1,qsize)] - [drgap,zeros(1,xsize-1)];

ineq_Info.c = [ineq_Info.c; cF; cB];
ineq_Info.cx = [ineq_Info.cx; cFx; cBx];
ineq_Info.cu = [ineq_Info.cu; zeros(1,usize); zeros(1, usize)];
ineq_Info.cy = [ineq_Info.cy; zeros(1,ysize); zeros(1, usize)];

end