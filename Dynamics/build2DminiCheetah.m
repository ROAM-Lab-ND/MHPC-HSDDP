function [robot, mc2D] = build2DminiCheetah()
% This function reformualtes the 3D mc inertia parameter to 2D, and
% creates a full planar quadruped model structure.
% The 0-configuration is with legs straight down, cheetah
% pointed along the +x axis of the ICS.
% The ICS has +z up, +x right, and +y inner page
% Planar model has 7 DoFs, x, z translation, rotation around y
% and front (back) hip and knee rotations

    mc3D = get3DMCParams();
    mc2D = get2DMCParams(mc3D);
    
    bodyMass            =  mc2D.bodyMass;
    bodyLength          =  mc2D.bodyLength;
    bodyHeight          =  mc2D.bodyHeight;
    bodyWidth           =  mc2D.bodyWidth;
    bodyCoM             =  mc2D.bodyCoM;
    bodyRotInertia      =  mc2D.bodyRotInertia;
    hipLinkLength       =  mc2D.hipLinkLength;
    hipLinkMass         =  mc2D.hipLinkMass;
    hipLinkCoM          =  mc2D.hipLinkCoM;
    hipRotInertia   =  mc2D.hipRotInertia;
    hipLoc              =  mc2D.hipLoc;
    kneeLinkLength      =  mc2D.kneeLinkLength;
    kneeLinkMass        =  mc2D.kneeLinkMass;
    kneeLinkCoM         =  mc2D.kneeLinkCoM;
    kneeRotInertia  =  mc2D.kneeRotInertia;
    
    %% model initilization
    robot.NB = 7;                                  % number of moving bodies (2 fictitious bodies for trunk translations)
    robot.parent  = zeros(1,robot.NB);             % parent body indices
    robot.Xtree   = repmat({eye(6)},robot.NB,1);   % coordinate transforms
    robot.jtype   = repmat({'  '},robot.NB,1);     % joint types
    robot.I       = repmat({zeros(6)},robot.NB,1); % spatial inertias
    robot.gravity = [0 0 -9.81]';              % gravity acceleration vec
    
    nb = 0;                                        % current body index
    
    %% trunk translation x (body num 1) (massless)
    nb = nb + 1;
    robot.parent(nb) = nb - 1;
    robot.Xtree{nb} = eye(1);
    robot.jtype{nb} = 'Px';
    robot.I{nb} = mcI(0, zeros(3,1), zeros(3,3));
    
    %% trunk translation z direction (body num 2) (massless)
    nb = nb + 1;
    robot.parent(nb) = nb - 1;
    robot.Xtree{nb} = eye(1);
    robot.jtype{nb} = 'Pz';
    robot.I{nb} = mcI(0, zeros(3,1), zeros(3,3));
    
    %% trunck rotation about y direction (body num 3)
    nb = nb + 1;
    robot.parent(nb) = nb - 1;
    robot.Xtree{nb} = eye(1);
    robot.jtype{nb} = 'Ry';
    robot.I{nb} = mcI(bodyMass, bodyCoM, bodyRotInertia);
    
    nbase = nb; % floating base index for attaching two children links (hip links)
    
    NLEGS = 2;
    for i = 1:NLEGS
        %% Hip link
        nb = nb + 1;
        robot.parent(nb) = nbase; % parent of the hip link is base
        robot.Xtree{nb} = plux(ry(0), hipLoc{i});  % translation (half the body length)
        robot.jtype{nb} = 'Ry';
        robot.I{nb} = mcI(hipLinkMass, hipLinkCoM, hipRotInertia);
        
        %% Knee link
        nb = nb + 1;
        robot.parent(nb) = nb - 1; % parent of the knee link is hip link
        robot.Xtree{nb} = plux( ry(0), mc2D.kneeLoc);    % translation (length of hip link)
        robot.jtype{nb} = 'Ry';
        robot.I{nb} = mcI(kneeLinkMass, kneeLinkCoM, kneeRotInertia);
    end
end