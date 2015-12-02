function LoadVehicleParams( Vehicle )
%Vehicle function is just here to make the Vehicle constructor prettier. 
%Our params are huge, so just put its construction in here. 

%We may not need all of these now, but it doesn't hurt to load everything. 


%%%%%%%%%%%%%%
% Load general car parameters
%%%%%%%%%%%%%%
Vehicle.params.control_delay =      0; %sec
Vehicle.params.stiffness =          120;
Vehicle.params.susp_conn_height =   -.8;
Vehicle.params.max_susp_force =     5812.4; %N
Vehicle.params.dampFactor =         10;
Vehicle.params.exp_dampFactor =     0;
Vehicle.params.roll_influence =     0;
Vehicle.params.steering_coeff =     -1.5;
Vehicle.params.max_steering =       20; %degrees
Vehicle.params.max_steering_rate =  13; %degrees/sec
Vehicle.params.accel_offset =       0; 
Vehicle.params.steering_offset =    0;
Vehicle.params.stall_torque_coeff = .13;
Vehicle.params.torque_speed_slope = 1.2685; %m/s/s

%%%%%%%%%%%%%%
% Create the body shape
%%%%%%%%%%%%%%

Vehicle.body.length =       2.7; %m
Vehicle.body.width =        2; %m
Vehicle.body.height =       1; %m
Vehicle.body.mass =         5; %kg
Vehicle.body.base_ht =      Vehicle.body.height/2; %m
Vehicle.body.restitution =  0; %m


%%%%%%%%%%%%%%
% Create the wheel shapes
%%%%%%%%%%%%%%

for i=1:4,
  Vehicle.wheel{i}.radius =        .5; %m
  Vehicle.wheel{i}.width =         .25; %m
  Vehicle.wheel{i}.mass =          1; %kg
  Vehicle.wheel{i}.restitution =   0;
  Vehicle.wheel{i}.dyn_friction =  .247; %µ_k
  Vehicle.wheel{i}.slip_coeff =    0;
  Vehicle.wheel{i}.traction_friction = 1000; %µ_s
  Vehicle.wheel{i}.side_friction = 1.56; %µ_s to the side
end 

%%%%%%%%%%%%%%
% Parameters dependent on other parts
%%%%%%%%%%%%%%
Vehicle.params.susp_rest_length =   .045; %m
Vehicle.params.max_susp_travel =    .045; %m

%Steering angle is with respect to the relative forward axis of the car.
Vehicle.state.steering = 0;
Vehicle.state.force = 0;
Vehicle.state.timecounter = 10000;
Vehicle.params.MagicB = 5;
Vehicle.params.MagicC = 1.65;
Vehicle.params.MagicE = -5;



end

