function OptimizerDemo
%A demo for showcasing the ease(?) of the Bullet-MATLAB simulator. 

%%%%%%%%%%%%%%%
%%%% CREATE THE BULLET WRAPPER
%%%%%%%%%%%%%%%

if exist('BulletSim'), 
  BulletOpti.delete;
end
BulletOpti = SETUP(true);

%%%%%%%%%%%%%%%
%%%% CREATE TERRAIN
%%%%%%%%%%%%%%%

Map = Terrain(100, 100, 0, 1);

%%%%%%%%%%%%%%%
%%%% CREATE THE VEHICLE
%%%%%%%%%%%%%%%
RayVehicles{1} = RaycastVehicle();

%%%%%%%%%%%%%%%
%%%% ADD OUR OBJECTS TO THE BULLET SIM
%%%%%%%%%%%%%%%
BulletOpti.AddTerrain(Map);
BulletOpti.AddRaycastVehicles(RayVehicles);

%%%%%%%%%%%%%%%
%%%% FIND A PATH FOR OUR CAR, AND OPTIMIZE IT.
%%%%%%%%%%%%%%%


% Notice: setting the angles here is a bit pointless, but necessary; if you're
% optimizing the path, the angle will change to fit the optimization. 
% poses = [x; y; th; vel]
poses = [ 5,     12;
          6,     8; 
          pi/2,  pi/2;
          0,     2];


% SetToGround is essential for an optimization routine.

BulletOpti.SetToGround(RayVehicles{1}, poses(1,1), poses(2,1));
[xs, ys, ths, major_poses] = SplineOptimize(Map, poses, true);

% Add2DPath gives us a guiding line on our 3D mesh, to tell us where we're
% supposed to go, compared to where the car actually is. 
Map.Add2DPath(xs, ys);

% Since our vehicle follows convention and sets the y-axis as forward, we have
% to shift the direction back by another pi/2 to set it towards the x-axis.
% We'll have to do the same when we take the residuals in GN_Optimize. 
% Look in TurnRightRound for this compensation term. 
first_thetas = ths{1};
RayVehicles{1}.TurnRightRound(first_thetas(1));

start_pose = RayVehicles{1}.GetPosition();
start_rot = RayVehicles{1}.GetRotation();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
waitforbuttonpress
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[engine_forces, steering_angles] = BulletOpti.GN_Optimize(RayVehicles{1}, ...
  major_poses, start_pose, start_rot, poses(4, :));

%%%%%%%%%%%%%%%
%%%% PASS COMMANDS TO THE CAR 
%%%% If there's a command chain, pass it through; if not, just generate random
%%%% points
%%%%%%%%%%%%%%%

for i = 1:numel(engine_forces),
  RayVehicles{1}.AddCommand(steering_angles{i}, engine_forces{i});
  e = engine_forces{i}
end

% %%%%%%%%%%%%%%%
% %%%% RUN THE GUI
% %%%%%%%%%%%%%%%

BulletOpti.InitSimulation();
while(BulletOpti.gui.quit==false), 
  BulletOpti.CommandRaycastVehicle(RayVehicles{1});
  BulletOpti.RunSimulation;
end

end
