function [ output_args ] = VehicleDemo( input_args )

BuckshotSim = BuckshotCreator(true);

Map = Terrain(100, 100, 4, 1);
RayVehicles{1} = RaycastVehicle();

BuckshotSim.AddTerrain(Map);
BuckshotSim.AddRaycastVehicles(RayVehicles);

% %%%%%%%%%%%%%%%
% %%%% PASS COMMANDS TO THE CAR 
% %%%% If there's a command chain, pass it through; if not, just generate random
% %%%% points

% SetToGround is essential for an optimization routine.
BuckshotSim.SetToGround(RayVehicles{1}, 0, 0);
engine_forces = repmat(10, 1, 30);
steering_angles = repmat(pi/6, 1, 30);
RayVehicles{1}.AddCommand(steering_angles, engine_forces);

runOpenGL = false;
BuckshotSim.RunSimulation(runOpenGL);

