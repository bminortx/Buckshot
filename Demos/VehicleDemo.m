function [ output_args ] = VehicleDemo( input_args )
%VEHICLEDEMO Summary of this function goes here
%   Detailed explanation goes here

%%%%%%%%%%%%%%%
%%%% CREATE THE BULLET WRAPPER
%%%%%%%%%%%%%%%

if exist('BulletSim'), 
  BulletOpti.delete;
end
BulletOpti = SETUP(true);

%%%%%%%%%%%%%%%
%%%% CREATE TERRAIN AND VEHICLE
%%%%%%%%%%%%%%%

Map = Terrain(100, 100, 4, 1);
RayVehicles{1} = RaycastVehicle();

%%%%%%%%%%%%%%%
%%%% ADD OUR OBJECTS TO THE BULLET SIM
%%%%%%%%%%%%%%%
BulletOpti.AddTerrain(Map);
BulletOpti.AddRaycastVehicles(RayVehicles);

% %%%%%%%%%%%%%%%
% %%%% PASS COMMANDS TO THE CAR 
% %%%% If there's a command chain, pass it through; if not, just generate random
% %%%% points
% %%%%%%%%%%%%%%%

% SetToGround is essential for an optimization routine.
BulletOpti.SetToGround(RayVehicles{1}, 0, 0);
engine_forces = repmat(10, 1, 30);
steering_angles = repmat(pi/6, 1, 30);
RayVehicles{1}.AddCommand(steering_angles, engine_forces);

% %%%%%%%%%%%%%%%
% %%%% RUN THE GUI
% %%%%%%%%%%%%%%%

BulletOpti.InitSimulation();
while(BulletOpti.gui.quit==false), 
  BulletOpti.CommandRaycastVehicle(RayVehicles{1});
  BulletOpti.RunSimulation;
end



end

