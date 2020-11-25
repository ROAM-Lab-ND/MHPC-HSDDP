classdef PlanarQuadruped < handle
    properties
        bodyMass
        bodyLength
        bodyHeight
        bodyWidth
        bodyCoM
        bodyRotInertia
        hipLinkLength
        hipLinkMass
        hipLinkCoM
        hipLinkRotInertia
        hipLoc
        kneeLinkLength
        kneeLinkMass
        kneeLinkCoM
        kneeLinkRotInertia
        model
    end
            
    methods 
        function Quad = PlanarQuadruped()            
        end
    end
    
    methods 
        function buildModel(Quad)
            % This function creates a full planar quadruped model structure
            % The 0-configuration is with legs straight down, cheetah
            % pointed along the +x axis of the ICS.
            % The ICS has +z up, +x right, and +y inner page
            % Planar model has 7 DoFs, x, z translation, rotation around y
            % and front (back) hip and knee rotations
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
            robot.I{nb} = mcI(Quad.bodyMass, Quad.bodyCoM, Quad.bodyRotInertia);
            
            nbase = nb; % floating base index for attaching two children links (hip links)
            
            for i = 1:NLEGS
                %% Hip link
                nb = nb + 1;
                robot.parent(nb) = nbase; % parent of the hip link is base
                robot.Xtree{nb} = plux(ry(0), Quad.hipLoc{i});  % translation (half the body length)
                robot.jtype{nb} = 'Ry';
                robot.I{nb} = mcI(Quad.hipLinkMass, Quad.hipLinkCoM, Quad.hipLinkRotInertia);
                
                %% Knee link
                nb = nb + 1;
                robot.parent(nb) = nb - 1; % parent of the knee link is hip link
                robot.Xtree{nb} = plux( ry(0), [0 0 -Quad.hipLinkLength]);    % translation (length of hip link)
                robot.jtype{nb} = 'Ry';
                robot.I{nb} = mcI(Quad.kneeLinkMass, Quad.kneeLinkCoM, Quad.kneeLinkRotInertia);
            end
            Quad.model = robot;
        end      
        
        
    end
end