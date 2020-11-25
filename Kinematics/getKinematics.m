function T = getKinematics(q)
% This function computes the homogeneous transformation of each joint-fixed
% frame w.r.t. Inertia coordinate system (IC)
% Link index convention: body 1, f hip 2, f knee 3, b hip 4, b knee 5
% q(1):x  q(2):y  q(3):roty
% q(4):theta1 hip  q(5):theta2 knee (front leg)
% q(6):theta1 hip  q(7):theta2 knee (back leg)          

% get link geometry parameters
mc3D = get3DMCParams();
mc2D = get2DMCParams(mc3D);
        
%% kinematics
% preallocate to save memory
T  =  repmat({eye(4)}, [5, 1]);

% body Pos and Orientation w.r.t IC
T{1}    =  Trans([q(1), 0, q(2)])*RotY(q(3));   

% body frame to front hip frame
T12     =  Trans(mc2D.hipLoc{1})*RotY(q(4));

% front hip frame to front knee frame
T23     =  Trans(mc2D.kneeLoc)*RotY(q(5));

% body frame to back hip frame
T14     =  Trans(mc2D.hipLoc{2})*RotY(q(6));

% back hip frame to back knee frame
T45     =  Trans(mc2D.kneeLoc)*RotY(q(7));

% front hip frame w.r.t IC
T{2}    =  T{1}*T12;

% front knee frame w.r.t IC
T{3}    =  T{2}*T23;

% back hip frame w.r.t IC
T{4}    = T{1}*T14;

% back knee frame w.r.t IC
T{5}    = T{4}*T45;
end