classdef RaycastVehicle < handle
  % RaycastVehicle is a special compound that Bullet itself puts together. 
  % Like its cousin, Vehicle, RaycastVehicle gets all of its parameters from 
  % LoadVehicleParameters, so has no arguments in its constructor. 
  %-------------------
  % Structs:
  %
  % - motionstate: RaycastVehicle saves its state over every DeltaTime 
  %     (specified by the user, but defaulted to every 30th of a second) in 
  %     this struct. It must be accessed with an index or a time. 
  %     It includes the following: 
  %       *timestep at which the state occurs
  %       *the position of the car in world space
  %       *the rotation of the car in world space
  %       *steering angle 
  %       *force on the car's back two wheels (ie acceleration)
  % - command: holds the current command going to the car. command, like
  %     motionstruct, is a sequence of arrays, and several commands can be 
  %     pushed to the structure at once. Once the command array has been 
  %     traversed, all variables are set to zero. It includes the following: 
  %       *the duration of the command
  %       *the steering angle
  %       *the force
  % Structs should be accessed through their Get() functions.
  %-------------------
  % Methods:
  %
  % - AddCommand(steering_angle, force, time): Adds a command to the 
  %     command cell array. For the duration of the time interval, the car will 
  %     pass the steering and force commands specified to the vehicle. 
  %     steering_angle is w.r.t. the forward axis; left is a positive radian 
  %     value, and right is a negative radian. Once the time is up, the vehicle 
  %     will either go to the next command in the queue, or set both 
  %     steering_angle and force to zero. 
  % - PushCommand(): pushes the latest series of commands to the Bullet Sim.
  %     PushCommand is used exclusively in bullet_interface for two reasons:
  %     1. It docks the timecounter that we use to determine when to change to
  %     the next command.
  %     2. It changes our command.index counter once that timer reaches 0. 
  %-------------------
  % Setters:
  %
  % - SetPosition([position]): sets the position of the car in world space
  % - SetSteering(radian_angle): sets the steering_angle (with infinite 
  %     duration). 
  % - SetForce(force): sets the force (with infinite duration). 
  %-------------------
  % Getters:
  % 
  % - GetMotionState('type', index): gets the motionstate specified through the
  %     arguments. 
  %     * 'type' = 'time' or 'index' 
  %     * index is the time specified, or the index of the motionstate in the
  %       cell array.
  % - GetCommand(index): gets the command at a certain index in the command cell
  %     array.
  
  properties
    params;
    state;
    body;
    wheel;
    type;
    id;
    motionstate;
    command;
  end
  
  methods
    %%%%%%%%%%%%%%%%%%
    %%% CONSTRUCTOR
    %%%%%%%%%%%%%%%%%%
    function this = RaycastVehicle()
      this.type = 'RaycastVehicle';
      this.id = 0;
      %%%Set up command structure
      this.command.duration = [];
      this.command.steering = [];
      this.command.force = [];
      this.command.index = 1; 
      
      LoadVehicleParams(this);
      
      %%%Set up the body
      this.body.color = [0,0,1];
      this.body.position = [0, 0, this.body.base_ht+this.body.height/2];
      this.body.rotation = [1,0,0; 0,1,0; 0,0,1];
      this.wheel{1}.color = [0,0,1];
      this.wheel{2}.color = [0,0,1];
      this.wheel{3}.color = [.5,.5,.5];
      this.wheel{4}.color = [.5,.5,.5];
      
      %%%Set up the motionstate structure
      this.motionstate.timestep = 0;
      this.motionstate.position = {this.body.position};
      this.motionstate.rotation = {this.body.rotation};
      this.motionstate.steering = {0};
      this.motionstate.force = {0};
      this.motionstate.lin_velocity = {0};
      this.motionstate.ang_velocity = {0};
      
      %%%Set up the wheels
      x=[-(this.wheel{1}.width/2);(this.wheel{1}.width/2)];
      angle=linspace(0,2*pi,40);
      [x,angle]=meshgrid(x,angle);
      y=this.wheel{1}.radius*cos(angle);
      z=this.wheel{1}.radius*sin(angle);
      for i=1:4,
        this.wheel{i}.x_vals = x;
        this.wheel{i}.y_vals = y;
        this.wheel{i}.z_vals = z;
      end
      axle_width = this.body.width*.5-(.3*this.wheel{1}.width);
      axle_length = this.body.length*.5;
      axle_height = this.body.height/2+this.params.susp_conn_height-...
        this.params.susp_rest_length;
      this.wheel{1}.position = [axle_width, axle_length, axle_height];
      this.wheel{1}.rotation = [0,-1,0; 1,0,0; 0,0,1];
      this.wheel{2}.position = [-axle_width, axle_length, axle_height];
      this.wheel{2}.rotation = [0,-1,0; 1,0,0; 0,0,1];
      this.wheel{3}.position = [-axle_width, -axle_length, axle_height];
      this.wheel{3}.rotation = [0,-1,0; 1,0,0; 0,0,1];
      this.wheel{4}.position = [axle_width, -axle_length, axle_height];
      this.wheel{4}.rotation = [0,-1,0; 1,0,0; 0,0,1];

    end
    
    %%%%%%%%%%%%%%%%%%
    %%% METHODS
    %%%%%%%%%%%%%%%%%%
    function [handle] = Draw(this)
      rotz = [0 -1 0; 1 0 0; 0 0 1];
      w = this.body.width/2;
      l = this.body.length/2;
      h = this.body.height/2;
      rotmat = this.body.rotation;
      points = [ w w -w -w w w -w -w; 
                 l -l -l l l -l -l l;
                 h h h h -h -h -h -h];
      points = (rotmat*points)+repmat(this.body.position', 1, ...
                                                            numel(points(1,:)));
      vertices = points';
      faces = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
      handle = patch('Vertices', vertices, 'Faces', faces, ...
        'FaceColor', this.body.color);
      for i=1:4,
        x = this.wheel{i}.x_vals;
        szx = size(x);
        y = this.wheel{i}.y_vals;
        z = this.wheel{i}.z_vals;
        x = reshape(x, 1, numel(x));
        y = reshape(y, 1, numel(y));
        z = reshape(z, 1, numel(z));
        point = [x; y; z];
        rotmat = this.wheel{i}.rotation;
        point = (rotz*(rotmat*point))+repmat(this.wheel{i}.position', 1,...
          numel(x(1,:)));
        x = reshape(point(1,:), szx);
        y = reshape(point(2,:), szx);
        z = reshape(point(3,:), szx);
        handle = [handle, surf(x,y,z, 'FaceColor', this.wheel{i}.color)];
      end
    end
    
    %%%%%%%%%
    
    function AddCommand(this, steering, force)
      this.command.steering = [this.command.steering, steering];
      this.command.force = [this.command.force, force];
    end
    
    %%%%%%%%%
    
    function command = PushCommand(this)
      index = this.command.index;
      command.steering = this.command.steering(index);
      command.force = this.command.force(index);
      this.command.index = this.command.index+1;
    end
    
    function [ out ] = NoMoreCommands(this, save_commands)
      out = false;
      if this.command.index > numel(this.command.steering),
        out = true;
        command.steering = 0;
        command.force = 0;
      end
      if save_commands == true, 
        disp('Saving MotionStates.mat and Commands.mat...');
        save_state = this.motionstate;
        save_command = this.command;
        save('MotionStates.mat', '-struct', 'save_state');
        save('Commands.mat', '-struct', 'save_command');
      end
    end
    
    function TurnRightRound(this, first_theta)
      current_rot = this.GetRotation();
      current_yaw = atan2(current_rot(2, 1), current_rot(1, 1));
      theta2d = first_theta-current_yaw;
      theta2d = theta2d-(pi/2);
      % Rotates about the z axis.
      rot_mat = [cos(theta2d) -sin(theta2d) 0;
                 sin(theta2d)  cos(theta2d) 0;
                 0             0            1];
      this.body.rotation = rot_mat*this.body.rotation;
    end
    
    
    
    %%%%%%%%%%%%%%%%%%
    %%% SETTERS 
    %%%%%%%%%%%%%%%%%%
    function SetSteering(this, steering)
      this.command.steering = steering;
      this.motionstate.steering = this.command.steering;
    end
    
    function SetForce(this, force)
      this.command.force = force;
      this.motionstate.force = this.command.force;
    end
    
    function SetID(this, id)
      this.id = id;
    end
    
    %%%%%%%%%
    %%% This function is really only used for drawing purposes.
    function SetTransform(this, body_position, body_rotation,...
          wheel_fl_pos, wheel_fl_rot,...
          wheel_fr_pos, wheel_fr_rot,...
          wheel_bl_pos, wheel_bl_rot,...
          wheel_br_pos, wheel_br_rot)
        this.body.position = body_position;
        this.body.rotation = body_rotation;
        this.wheel{1}.position = wheel_fl_pos;
        this.wheel{2}.position = wheel_fr_pos;
        this.wheel{3}.position = wheel_bl_pos;
        this.wheel{4}.position = wheel_br_pos;
        this.wheel{1}.rotation = wheel_fl_rot;
        this.wheel{2}.rotation = wheel_fr_rot;
        this.wheel{3}.rotation = wheel_bl_rot;
        this.wheel{4}.rotation = wheel_br_rot;
    end
    
    %%%%%%%%%
    
    function SetPosition(this, position)
        old_pose = this.body.position;
        %Move the body
        pose_diff = position - old_pose;
        pose_diff(3) = pose_diff(3)+this.body.height/2;
        this.body.position = position;
        %Move the wheels
        for i = 1:4,
          new_pose = this.wheel{i}.position+pose_diff;
          this.wheel{i}.position = new_pose;
        end
        this.motionstate.position = [this.body.position];
        this.motionstate.rotation = [this.body.rotation];
    end
    
    %%%%%%%%
    
    function SetPosAndRot(this, position, rotation)
        old_pose = this.body.position;
        %Move the body
        pose_diff = position - old_pose;
        pose_diff(3) = pose_diff(3)+this.body.height/2;
        this.body.position = position;
        %Move the wheels
        for i = 1:4,
          new_pose = this.wheel{i}.position+pose_diff;
          this.wheel{i}.position = new_pose;
        end
        this.body.rotation = rotation;

        this.motionstate.position = [this.body.position];
        this.motionstate.rotation = [this.body.rotation];
    end
    
    %%%%%%%%%
    
    function SetMotionState(this, time, position, rotation, ...
        steering, force, lin_velocity, ang_velocity)
      this.motionstate.timestep = [this.motionstate.timestep; {time}];
      this.motionstate.position = [this.motionstate.position; {position}];
      this.motionstate.rotation = [this.motionstate.rotation; {rotation}];
      this.motionstate.steering = [this.motionstate.steering; {steering}];
      this.motionstate.force = [this.motionstate.force; {force}];
      this.motionstate.lin_velocity = [this.motionstate.lin_velocity; {lin_velocity}];
      this.motionstate.ang_velocity = [this.motionstate.ang_velocity; {ang_velocity}];
    end
    
    %%%%%%%%%%%%%%%%%%
    %%% GETTERS 
    %%%%%%%%%%%%%%%%%%
    
    %%%Since the RaycastVehicle doesn't actually use real collision shapes to
    %%%create the collisions (just raycasts), we have to pass all of the
    %%%parameters through to the constructor in Bullet. 
    
    function parameters = GetParameters(this)
      parameters = [this.body.length,...
        this.body.width,...
        this.body.height,...
        this.wheel{1}.dyn_friction,...
        this.wheel{1}.side_friction,...
        this.wheel{1}.slip_coeff,...
        this.params.control_delay,...
        this.body.mass,...
        this.wheel{1}.radius,...
        this.wheel{1}.width,...
        this.wheel{1}.traction_friction,...
        this.params.susp_conn_height,...
        this.params.stiffness,...
        this.params.max_susp_force,...
        this.params.max_susp_travel,...
        this.params.susp_rest_length,...
        this.params.dampFactor,...
        this.params.exp_dampFactor,...
        this.params.roll_influence,...
        this.params.steering_coeff,...
        this.params.max_steering,...
        this.params.max_steering_rate,...
        this.params.accel_offset,...
        this.params.steering_offset,...
        this.params.stall_torque_coeff,...
        this.params.torque_speed_slope,...
        this.params.MagicB,...
        this.params.MagicC,...
        this.params.MagicE];
    end 
    
    %%%%%%%%%
    
    function steering = GetSteering(this)
      steering = this.command.steering(this.command.index);
    end
    
    function force = GetForce(this)
      force = this.command.force(this.command.index);
    end

    function id = GetID(this)
      id = this.id;
    end
    
    function type = GetType(this)
      type = this.type;
    end    
    
    function position = GetPosition(this)
      position = this.body.position;
    end
    
    function rotation = GetRotation(this)
      rotation = this.body.rotation;
    end
    
    %%%%%%%%%
    
    function motionstate = GetMotionState(type, index)
      if strcmp(type, 'time')
        ind = find(this.motionstate.time, index);
        motionstate.timestep = this.motionstate.timestep(ind);
        motionstate.position = this.motionstate.position(ind,:);
        motionstate.rotation = this.motionstate.rotation(ind,:);
        motionstate.steering = this.motionstate.steering(ind); 
        motionstate.force = this.motionstate.force(ind);
        motionstate.lin_velocity = this.motionstate.lin_velocity(ind);
        motionstate.ang_velocity = this.motionstate.ang_velocity(ind);
      elseif strcmp(type, 'index'),
        motionstate.timestep = this.motionstate.timestep(index);
        motionstate.position = this.motionstate.position(index,:);
        motionstate.rotation = this.motionstate.rotation(index,:);
        motionstate.steering = this.motionstate.steering(index); 
        motionstate.force = this.motionstate.force(index);
        motionstate.lin_velocity = this.motionstate.lin_velocity(index);
        motionstate.ang_velocity = this.motionstate.ang_velocity(index);
      else
        disp('GetMotionState: undefined parameter passed.');
        disp('--First argument must be "time" or "index".');
      end
    end
    
  end
  
end

