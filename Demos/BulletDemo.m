function BulletDemo
%%% A demo for showcasing the ease(?) of the Bullet-MATLAB simulator. 

%%% 1. CREATE THE BULLET WRAPPER
%%% The most important part; that's why you're using this, right?

if exist('BuckshotSim'), 
  BuckshotSim.delete;
end
BuckshotSim = SETUP(true);

%%% CREATE TERRAIN
Map = Terrain(50, 50, 3, 1);

%%% CREATE SHAPES 
%%% It's a good practice to place your shapes in a cell matrix, for
%%% two reasons:
%%% A. It's easier to initialize them all at one time,
%%% B. They are easier to access when adding constraints
%%% Directory: Matlab_shapes
% Shapes{1} = Sphere(3, 5, 2);
% Shapes{1}.SetPosition([10, 10, 8]);
% Shapes{2} = Cube(10, 10, 1, 1, 2);
% Shapes{2}.SetPosition([0, 0, 2]);
% Shapes{3} = Sphere(1, 5, 2);
% Shapes{3}.SetPosition([-10, 10, 8]);
% Shapes{4} = Sphere(1, 5, 2);
% Shapes{4}.SetPosition([-6, 8, 8]);

%%% CREATE CONSTRAINTS Place your constraints in a cell matrix, for
%%% the same reason you did it for your shapes.  Directory:
%%% Matlab_constraints.  NB - Constraints often have more than one
%%% constructor, depending on the number of shapes you want to
%%% contrain, or the type of data you want to use to identify the
%%% constraint.
% Constraints{1} = Hinge(Shapes{1}, [4, 0, 0], [0, 1, 0]);
% Constraints{1}.SetLimits(3, -3);
% Constraints{2} = SixDOF(Shapes{3}, Shapes{4}, ...
%                         [1, 1, 0, 1, 0, 1], [-1, 1, 0, -1, 0, -1]);
% Constraints{3} = PointToPoint(Shapes{3},[1, 1, 1]);

%%%% ADD OUR OBJECTS TO BUCKSHOT
BuckshotSim.AddTerrain(Map);
% BuckshotSim.AddShapes(Shapes);
% BuckshotSim.AddConstraints(Constraints);

% %%%% RUN THE GUI
% BuckshotSim.InitSimulation();
% while(BuckshotSim.gui.quit==false), 
%   BuckshotSim.RunSimulation;
% end
