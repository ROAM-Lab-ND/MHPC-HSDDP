function  robot = cartpole( mass_cart, mass_pole, length_pole, g )

robot.NB = 2;
robot.parent = [0:1];

robot.jtype{1} = 'Px'; % The first joint is a prismatic in the +X0 direction
robot.jtype{2} = 'Rz'; % The second joint is a revolute in the +Z1 direction

robot.Xtree{1} = eye(1); % The transform from {0} to just before joint 1 is identity
robot.Xtree{2} = eye(1); % The transfrom from {1} to just before joint 2 is identity

robot.I{1} = mcI( mass_cart, [0 0 0], 0 );
robot.I{2} = mcI( mass_pole, [0 -length_pole 0], 0 );

robot.gravity = [0 -g 0]';

