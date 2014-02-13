classdef bullet_interface < handle
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%% MATLAB class wrapper to the Bullet C++ architecture.
  %%%% Any method with 'bullet_interface_mex(...)'
  %%%% represents a call to the Bullet wrapper for MATLAB.
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  
  properties (SetAccess = public)
    bulletHandle; % Handle to the underlying C++ class instance
    %These are arrays that hold the different objects we have created.
    Terrain;
    Shapes;
    Compounds;
    Constraints;
    RayVehicles;
    %This is the structure that holds our gui decisions;
    gui;
  end
  
  methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%
    %%%% CONSTRUCTOR/DESTRUCTOR
    %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function this = bullet_interface(varargin)
      this.bulletHandle = bullet_interface_mex('new', varargin{:});
      this.Terrain = [];
      this.gui.draw = false;           % bool to draw Sim
      this.gui.quit = false;           % bool to quit Sim
      this.gui.fig = [];               % handle that holds Sim figs
      this.gui.simfig = [];               % handle that holds our optimizer paths
      this.gui.iter = false;           % step one frame at a time
      this.gui.path = false;           % trace shape paths (not polished)
      this.gui.draw_constraint = true; % draw our constraints
      this.gui.timestep = 0;           % what timestep we're at
      this.gui.framerate = 0;          % fps
      this.gui.counter = 0;            % These variables
      this.gui.c1 = clock;                % help calculate
      this.gui.c2 = 0;                    % our
      this.gui.seconds = 0;               % framerate
    end
    
    function delete(this)
      bullet_interface_mex('delete', this.bulletHandle);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%
    %%%% ADDING OBJECTS
    %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function [ids] = AddShapes(this, ShapeArray)
      ids = [];
      if isempty(this.Shapes),
        this.Shapes = ShapeArray;
      else
        this.Shapes = [this.Shapes ShapeArray];
      end
      for i = 1:numel(this.Shapes),
        Shape = this.Shapes{i};
        type = Shape.GetType();
        % AddCube - Adds a bullet cube to the world.
        if strcmp(type, 'Cube')==true,
          CubeDim = Shape.GetDim();
          mass = Shape.GetMass();
          restitution = Shape.GetRestitution();
          position = Shape.GetPosition();
          rotation = Shape.GetRotation();
          id = bullet_interface_mex('AddShape', ...
            this.bulletHandle, type, CubeDim(1), CubeDim(2), CubeDim(3), ...
            mass, restitution, position, rotation);
          Shape.SetID(id);
        end
        % AddSphere - Adds a bullet sphere to the world.
        if strcmp(type, 'Sphere')==true,
          radius = Shape.GetRadius();
          mass = Shape.GetMass();
          restitution = Shape.GetRestitution();
          position = Shape.GetPosition();
          rotation = Shape.GetRotation();
          id = bullet_interface_mex('AddShape', ...
            this.bulletHandle, type, radius, mass, restitution, ...
            position, rotation);
          Shape.SetID(id);
        end
        % AddCylinder - Adds a bullet cylinder to the world.
        if strcmp(type, 'Cylinder')==true,
          radius = Shape.GetRadius();
          height = Shape.GetHeight();
          mass = Shape.GetMass();
          restitution = Shape.GetRestitution();
          position = Shape.GetPosition();
          rotation = Shape.GetRotation();
          id = bullet_interface_mex('AddShape', ...
            this.bulletHandle, type, radius, height, mass,...
            restitution, position, rotation);
          Shape.SetID(id);
        end
        ids = [ids, id];
      end
    end
    
    %%%%%%%%%%%%%
    
    function AddTerrain(this, Terrain)
      %Adds a randomly-generated terrain to the bullet environment
      if numel(this.Terrain) > 0,
        disp('We already have a terrain. Delete the class to create a new one.');
        return;
      end
      this.Terrain = Terrain;
      heightmap = Terrain.GetHeightmap();
      grad = Terrain.GetGranularity();
      extrema = Terrain.GetDim();
      min_ht = extrema(3, 1);
      max_ht = extrema(3, 2);
      sze = size(heightmap{1});
      normal = Terrain.GetNormal();
      id = bullet_interface_mex('AddTerrain', ...
        this.bulletHandle, sze(1), sze(2), grad, min_ht, max_ht, heightmap{1}, ...
        heightmap{2}, heightmap{3}, normal);
      this.Terrain.SetID(id);
    end
    
    %%%%%%%%%%%%%
    
    function AddCompounds( this, Compounds  )
      % AddCompunds - Adds a bullet compound to the world. Is just an
      % amalgamation of Shapes and Constraints, so we just use those arguments.
      if isempty(this.Compounds),
        this.Compounds = Compounds;
      else
        this.Compounds = [this.Compounds Compounds];
      end
      for i=1:numel(Compounds),
        Shape_ids = this.AddShapes(Compounds{i}.GetShapes());
        Con_ids = this.AddConstraints(Compounds{i}.GetConstraints());
        Type = Compounds{i}.GetType();
        disp(Type);
        id = bullet_interface_mex('AddCompound', this.bulletHandle, ...
          Type, Shape_ids, Con_ids);
        Compounds{i}.SetID(id);
      end
    end
    
    %%%%%%%%%%%%%
    
    function AddRaycastVehicles( this, RayVehicles  )
      if isempty(this.RayVehicles),
        this.RayVehicles = RayVehicles;
      end
      for i=1:numel(RayVehicles),
        parameters = RayVehicles{i}.GetParameters();
        position = RayVehicles{i}.GetPosition();
        rotation = RayVehicles{i}.GetRotation();
        id = bullet_interface_mex('AddRaycastVehicle', this.bulletHandle, ...
          parameters, position, rotation);
        RayVehicles{i}.SetID(id);
      end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%
    %%%% ADDING CONSTRAINTS
    %%%% These are ways to conveniently link two shapes through a constraint.
    %%%% We got all sorts of choices here...
    %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function [ids] = AddConstraints(this, Constraints)
      ids = [];
      if isempty(this.Compounds),
        this.Constraints = Constraints;
      else
        this.Constraints = [this.Constraints Constraints];
      end
      for i=1:numel(Constraints),
        Constraint = Constraints{i};
        type = Constraint.GetType();
        
        % Point-to-point
        if strcmp(type, 'PointToPoint_one')==true,
          id_A = Constraint.Shape_A.GetID();
          pivot_in_A = Constraint.pivot_in_A;
          id = bullet_interface_mex('AddConstraint', this.bulletHandle, ...
            'PointToPoint_one', id_A, pivot_in_A);
        elseif strcmp(type, 'PointToPoint_two')==true,
          id_A = Constraint.Shape_A.GetID();
          pivot_in_A = Constraint.pivot_in_A;
          id_B = Constraint.Shape_B.GetID();
          pivot_in_B = Constraint.pivot_in_B;
          id = bullet_interface_mex('AddConstraint', this.bulletHandle, ...
            'PointToPoint_two', id_A, id_B, pivot_in_A, pivot_in_B);
          
          % Hinge
        elseif strcmp(type, 'Hinge_one_transform')==true,
          id_A = Constraint.Shape_A.GetID();
          transform_A = Constraint.transform_A;
          limits = Constraint.GetLimits();
          id = bullet_interface_mex('AddConstraint', this.bulletHandle, ...
            'Hinge_one_transform', id_A, transform_A, limits);
          
        elseif strcmp(type, 'Hinge_two_transform')==true,
          id_A = Constraint.Shape_A.GetID();
          transform_A = Constraint.transform_A;
          id_B = Constraint.Shape_B.GetID();
          transform_B = Constraint.transform_B;
          limits = Constraint.GetLimits();
          id = bullet_interface_mex('AddConstraint', this.bulletHandle, ...
            'Hinge_two_transform', id_A, id_B, transform_A, transform_B, limits);
          
        elseif strcmp(type, 'Hinge_one_pivot')==true,
          id_A = Constraint.Shape_A.GetID();
          pivot_in_A = Constraint.pivot_in_A;
          axis_in_A = Constraint.axis_in_A;
          limits = Constraint.GetLimits();
          id = bullet_interface_mex('AddConstraint', this.bulletHandle, ...
            'Hinge_one_pivot', id_A, pivot_in_A, axis_in_A, limits);
          
        elseif strcmp(type, 'Hinge_two_pivot')==true,
          id_A = Constraint.Shape_A.GetID();
          pivot_in_A = Constraint.pivot_in_A;
          axis_in_A = Constraint.axis_in_A;
          id_B = Constraint.Shape_B.GetID();
          pivot_in_B = Constraint.pivot_in_B;
          axis_in_B = Constraint.axis_in_B;
          limits = Constraint.GetLimits();
          id = bullet_interface_mex('AddConstraint', this.bulletHandle, ...
            'Hinge_two_pivot', id_A, id_B, pivot_in_A, pivot_in_B, ...
            axis_in_A, axis_in_B, limits);
          
          % Hinge2
        elseif strcmp(type, 'Hinge2')==true,
          id_A = Constraint.Shape_A.GetID();
          id_B = Constraint.Shape_B.GetID();
          Anchor = Constraint.Anchor;
          Axis_1 = Constraint.Axis_1;
          Axis_2 = Constraint.Axis_2;
          damping = Constraint.GetDamping;
          stiffness = Constraint.GetStiffness;
          steering_angle = Constraint.GetSteeringAngle;
          id = bullet_interface_mex('AddConstraint', this.bulletHandle, ...
            'Hinge2', id_A, id_B, Anchor, Axis_1, ...
            Axis_2, damping, stiffness, steering_angle);
          
          % Six DOF
        elseif strcmp(type, 'SixDOF_one')==true,
          id_A = Constraint.Shape_A.GetID();
          transform_A = Constraint.GetTransform_A();
          limits = Constraint.GetLimits();
          id = bullet_interface_mex('AddConstraint', this.bulletHandle, ...
            'SixDOF_one', id_A, transform_A, limits);
        elseif strcmp(type, 'SixDOF_two')==true,
          id_A = Constraint.Shape_A.GetID();
          transform_A = Constraint.GetTransform_A();
          id_B = Constraint.Shape_B.GetID();
          transform_B = Constraint.GetTransform_B();
          limits = Constraint.GetLimits();
          id = bullet_interface_mex('AddConstraint', this.bulletHandle, ...
            'SixDOF_two', id_A, id_B, transform_A, transform_B, limits);
        end
        Constraint.SetID(id);
        ids = [ids, id];
      end
      
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%
    %%%% COMPOUND METHODS
    %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function CommandCompound(this, Compound)
      if isa(Compound, 'Vehicle'),
        steering = Compound.GetSteering();
        force = Compound.GetForce();
        id = Compound.GetID();
        bullet_interface_mex('CommandCompound', this.bulletHandle, 'Vehicle', ...
          id, steering, force);
      end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%
    %%%% RAYCAST VEHICLE METHODS
    %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function CommandRaycastVehicle(this, Vehicle)
      if this.gui.draw,
        command = Vehicle.PushCommand();
        id = Vehicle.GetID();
        bullet_interface_mex('CommandRaycastVehicle', this.bulletHandle, ...
          id, command.steering, command.force);
      end
    end
    
    % Used in StepSimulation
    function [steering, force, lin_vel, ang_vel] = GetMotionState(this, Vehicle)
      id = Vehicle.GetID();
      [steering, force, lin_vel, ang_vel] = ...
        bullet_interface_mex('GetMotionState', this.bulletHandle, id);
    end
    
    function SetToGround(this, Vehicle, x_coord, y_coord)
      id = Vehicle.GetID();
      % Get our new position
      [position] = bullet_interface_mex('SetToGround', this.bulletHandle, id, ...
        x_coord, y_coord);
      % Set our new position
      [body_position, body_rotation,...
        wheel_fl_pos, wheel_fl_rot,...
        wheel_fr_pos, wheel_fr_rot,...
        wheel_bl_pos, wheel_bl_rot,...
        wheel_br_pos, wheel_br_rot] = bullet_interface_mex('GetTransform', ...
        this.bulletHandle, 'RaycastVehicle', Vehicle.GetID());
      Vehicle.SetTransform(body_position, body_rotation,...
        wheel_fl_pos, wheel_fl_rot,...
        wheel_fr_pos, wheel_fr_rot,...
        wheel_bl_pos, wheel_bl_rot,...
        wheel_br_pos, wheel_br_rot);
    end
    
    function [states, end_position, end_rotation, end_lin_vel, end_ang_vel, grounded] = SpeedSim(this, Vehicle,...
        start_pose, start_rot, start_lin_vel, start_ang_vel, engine_commands, steering_commands)
      id = Vehicle.GetID();
      [states, end_position, end_rotation, end_lin_vel, end_ang_vel, grounded] = bullet_interface_mex('SpeedSim',...
        this.bulletHandle, id, start_pose, start_rot, start_lin_vel, start_ang_vel, engine_commands, steering_commands, ...
        numel(steering_commands));
    end
    
    function ResetVehicle(this, Vehicle, start_pose, start_rot)
      id = Vehicle.GetID();
      bullet_interface_mex('ResetVehicle', this.bulletHandle, id, start_pose, start_rot);
    end
    
    function RapidGUI(this, Vehicle, forces, steering_angles)
      Vehicle.AddCommand(steering_angles, forces);
      this.InitSimulation();
      while(this.gui.quit==false),
        this.CommandRaycastVehicle(Vehicle);
        this.RunSimulation;
      end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%
    %%%% OPTIMIZATION METHODS
    %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function [ J ] = GetJacobians(this, Vehicle, force, steering, ...
        start_pose, start_rot, start_lin_vel, start_ang_vel,...
        end_pos, end_rot, end_lin_vel, pos1, pos2)
      delta = 1e-9;
      J = zeros(4, 4);
      
      %Find the original difference
      orig_resid = this.GetResiduals(pos2, end_pos, end_rot, end_lin_vel);
      
      %Change each of our spline parameters by delta...
      force_change = force+delta;
      
      mod_pos = repmat(pos2, 1, 3);
      for i=1:3, 
        mod_pos(i,i) = mod_pos(i,i)+delta;
      end
      mod_steering = {};  
      speedsim_pos = {};
      speedsim_rot = {};
      speedsim_vel = {};
      
      for i=1:3, 
        [ ~, mod_steering{i} ] = RunSplineFunction(Vehicle, pos1, mod_pos(:,i));
      end
      
      % Run our four different paths
      
      %force
      [sim1, speedsim_pos{1}, speedsim_rot{1}, speedsim_vel{1}, ~, ~] = this.SpeedSim(Vehicle, start_pose,...
        start_rot, start_lin_vel, start_ang_vel, force_change, steering);
%       xf
      [sim2, speedsim_pos{2}, speedsim_rot{2}, speedsim_vel{2}, ~, ~] = this.SpeedSim(Vehicle, start_pose,...
        start_rot, start_lin_vel, start_ang_vel, force, mod_steering{1});
%       yf
      [sim3, speedsim_pos{3}, speedsim_rot{3}, speedsim_vel{3}, ~, ~] = this.SpeedSim(Vehicle, start_pose,...
        start_rot, start_lin_vel, start_ang_vel, force, mod_steering{2});
%       thf
      [sim4, speedsim_pos{4}, speedsim_rot{4}, speedsim_vel{4}, ~, ~] = this.SpeedSim(Vehicle, start_pose,...
        start_rot, start_lin_vel, start_ang_vel, force, mod_steering{3});
      % Now fill up our Jacobian matrix: output params x spline param
      for i=1:numel(speedsim_pos),
        sim_resid = this.GetResiduals(pos2, speedsim_pos{i}, speedsim_rot{i}, speedsim_vel{i});
        J(:, i) = (orig_resid-sim_resid)/delta;
      end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function [ resid ] = GetResiduals(this, pos2, end_pos, end_rot, end_lin_vel)
      % Residuals consist of x, y, theta (i.e. yaw), and final lin_vel
      resid = zeros(4, 1);
      resid(1) = pos2(1) - end_pos(1);
      resid(2) = pos2(2) - end_pos(2);
      new_yaw = atan2(end_rot(2,1), end_rot(1,1));
      resid(3) = pos2(3) - (new_yaw + pi/2);
      resid(4) = pos2(4) - norm(abs(end_lin_vel));
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function [norm_Deltas, Deltas] = GaussNewton3D(this, J, Resid)
      %%Gauss-Newton matrix multiplication.
      A = (J'*J);
      B = J'*Resid;
      Deltas = pinv(A)*B;
      damp = .5;
%       if max(abs(Deltas))>.1,
%         Deltas = Deltas*damp;
%       end
      norm_Deltas = norm(abs(Deltas), 2);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function [ engine_commands, steering_commands ] = GN_Optimize(this, Vehicle,...
        major_poses, start_pose, start_rot, velocities )
      
      %%% Initialize Variables
      norm_Deltas = Inf;
      jj = 0;
      maxIter = 200;
      hit_min = 0;
      c1 = clock;
      Old_resid = Inf;
      engine_commands = {};
      steering_commands = {};
      h = [];
      start_lin_vel = [0, 0, 0];
      start_ang_vel = [0, 0, 0];
      orig_start_pose = start_pose;
      orig_start_rot = start_rot;
      
      for k=1:numel(major_poses(1,:))-1,
        pos1 = [major_poses(:, k); velocities(k)];
        pos2 = [major_poses(:, k+1); velocities(k+1)];
        sim_pos2 = pos2;
        [force,~,~] = RunSplineFunction(Vehicle, pos1, sim_pos2);
        while norm_Deltas > 1e-5 && jj < maxIter,
%           disp(['force = ', num2str(force(1))]);
          % Figure out the commands we need to pass
          [~, steering_ang, Knots] = RunSplineFunction(Vehicle, pos1, sim_pos2);
          % Pass commands; gives an initial estimate of what we need
          [sim_states, end_pos, end_rot, end_lin_vel, ~, grounded] = this.SpeedSim(Vehicle, start_pose,...
            start_rot, start_lin_vel, start_ang_vel, force, steering_ang);
          
          % draw what we've done. 
          set(h ,'Color', 'c');
%           fprintf('there are %d states', numel(sim_states(1,:)));
          h = [h plot3(sim_states(1,:), sim_states(2,:), sim_states(3,:), 'b.-')];
          hh = plot(Knots(1,:), Knots(2,:), 'r*-', 'MarkerSize', 30);
          axis tight;
          drawnow;
          
          %%% Calculate Jacobians
          J = this.GetJacobians(Vehicle, force, steering_ang, start_pose, start_rot, ...
            start_lin_vel, start_ang_vel, end_pos, end_rot, end_lin_vel, pos1, pos2);
          %%%Calculate residuals 
          Resid = this.GetResiduals(pos2, end_pos, end_rot, end_lin_vel);
          %%% Calculate Gauss-Newton --> Deltas
          [norm_Deltas, Deltas] = this.GaussNewton3D(J, Resid);
%           waitforbuttonpress;
          % If we're on the ground...
          if grounded==1,
            %%% Add deltas to the commands
            force = force + Deltas(1);
            sim_pos2(1) = sim_pos2(1) + Deltas(2);
            sim_pos2(2) = sim_pos2(2) + Deltas(3);
            sim_pos2(3) = sim_pos2(3) + Deltas(4);
          else
            % If we flipped...
            norm_Deltas = 100;
            force = force-1;
            if force<=0, 
              force = .1;
            end
          end

          jj = jj+1;
          
          if sum(Resid) < sum(Old_resid),
            hit_min = hit_min+1;
          else
            Old_resid = Resid;
            hit_min = 0;
          end
          
          % A small counter to tell us how long this actually takes.
          c2 = clock;
          if etime(c2, c1)>1,
            disp('.');
            c1 = clock;
          end
          
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          
%           waitforbuttonpress;
          delete(hh);
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        % At the end of all this, we should have some successful commands
        engine_commands{k} = force;
        [~, steering_ang] = RunSplineFunction(Vehicle, pos1, sim_pos2);
        steering_commands{k} = steering_ang;
        
        % If our optimization worked above, our end points should be damn close
        % to where we should be next point.
        [sim_states, end_pos, end_rot, end_lin_vel, end_ang_vel, grounded] = this.SpeedSim(Vehicle, start_pose,...
            start_rot, start_lin_vel, start_ang_vel, force, steering_ang); 
        set(h ,'Color', 'c');
        h = [h plot3(sim_states(1,:), sim_states(2,:), sim_states(3,:), 'b.-')];
        hh = plot(Knots(1,:), Knots(2,:), 'r*-', 'MarkerSize', 30);
        axis tight;
        drawnow;
        start_pose = end_pos;
        start_rot = end_rot;
        start_lin_vel = end_lin_vel;
        start_ang_vel = end_ang_vel;
        waitforbuttonpress;
        norm_Deltas = Inf;
        jj = 0;
        
      end
      
      this.ResetVehicle(Vehicle, orig_start_pose, orig_start_rot);
      
    end
   
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%
    %%%% INITIALIZE THE GUI
    %%%% Code modified from RPI's MATLAB-only simulator.
    %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function InitSimulation(this)
      
      % Keypress event from run()
      function this = figure_keypress(~, event, this)
        switch event.Key
          case 'i'
            this.gui.draw = false;
            this.gui.iter = true;
          case 'q'
            this.gui.quit = true;
            close(this.gui.fig);
          case 'space'
            this.gui.draw = ~this.gui.draw;
          case 'p'
            this.gui.path = ~this.gui.path;
          case 'c'
            this.gui.draw_constraint = ~this.gui.draw_constraint;
        end
      end
      
      disp('Starting simulation...');
      disp(' || space: start / stop simulation');
      disp(' || i:     iterate a single step');
      disp(' || p:     trace the path of an object');
      disp(' || c:     turn off/turn on constraint display');
      disp(' || q:     quit');
      disp('----------------------------------------------');
      
      set(0, 'DefaultFigurePosition', [10,10,900,900]);
      this.gui.fig = figure;
      set(this.gui.fig, 'KeyPressFcn', {@figure_keypress, this});
      view(3);
      hold all;
      xlabel('x'); ylabel('y'); zlabel('z');
      title(['Bullet simulation (paused)', char(10), 'Timestep: ',...
        num2str(this.gui.timestep), char(10), 'Framerate: ', ...
        num2str(this.gui.framerate)]);
      for i = 1:numel(this.Terrain),
        this.Terrain(i).Draw;
      end
      this.DrawSimulation();
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%
    %%%% RUNNING THE SIMULATION
    %%%% Handles the interaction between the Bullet Physics engine and the Sim.
    %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    function RunSimulation(this)
      if this.gui.draw,
        title(['Bullet simulation (running)',char(10),'Timestep: ',...
          num2str(this.gui.timestep), char(10), 'Framerate: ', ...
          num2str(this.gui.framerate), '  fps']);
        this.StepSimulation();
      elseif this.gui.iter == true,
        title(['Bullet simulation (paused)',char(10),'Timestep: ',...
          num2str(this.gui.timestep), char(10), 'Framerate: ', ...
          num2str(this.gui.framerate), '  fps']);
        this.StepSimulation();
        this.gui.iter = false;
      else
        title(['Bullet simulation (paused)',char(10),'Timestep: ',...
          num2str(this.gui.timestep), char(10),'Framerate: ', ...
          num2str(this.gui.framerate), '  fps']);
      end
      
      this.DrawSimulation();
      %       if this.gui.quit == true,
      %         break;
      %       end
    end
    
    %%%% Updates all of our objects according to the Bullet world
    function StepSimulation(this)
      bullet_interface_mex('StepSimulation', this.bulletHandle);
      for i = 1:numel(this.Shapes),
        [position, rotation] = bullet_interface_mex('GetTransform', ...
          this.bulletHandle, 'Shape', this.Shapes{i}.GetID());
        this.Shapes{i}.SetTransform(position, rotation);
      end
      for i = 1:numel(this.RayVehicles),
        % If there are no more commands, then stop the simulation
        if this.RayVehicles{i}.NoMoreCommands(false) == true,
          if this.gui.draw == true,
            disp('RaycastVehicle ran out of commands.');
            disp('Sim is paused. Press spacebar to start the Sim again.');
          end
          this.gui.draw = false;
        else
          [body_position, body_rotation,...
            wheel_fl_pos, wheel_fl_rot,...
            wheel_fr_pos, wheel_fr_rot,...
            wheel_bl_pos, wheel_bl_rot,...
            wheel_br_pos, wheel_br_rot] = bullet_interface_mex('GetTransform', ...
            this.bulletHandle, 'RaycastVehicle', this.RayVehicles{i}.GetID());
          this.RayVehicles{i}.SetTransform(body_position, body_rotation,...
            wheel_fl_pos, wheel_fl_rot,...
            wheel_fr_pos, wheel_fr_rot,...
            wheel_bl_pos, wheel_bl_rot,...
            wheel_br_pos, wheel_br_rot);
          [steering, force, lin_vel, ang_vel] = GetMotionState(this, this.RayVehicles{i});
          this.RayVehicles{i}.SetMotionState(this.gui.timestep*(1/30), ...
            body_position, body_rotation, steering, force, lin_vel, ang_vel);
        end
      end
      for i = 1:numel(this.Constraints),
        if strcmp(this.Constraints{i}.GetType(), 'Hinge2')==true,
          [position] = bullet_interface_mex('GetTransform', ...
            this.bulletHandle, 'Constraint', this.Constraints{i}.GetID());
          this.Constraints{i}.SetPosition(position);
        end
      end
      this.gui.c2 = clock;
      this.gui.timestep = this.gui.timestep+1;
      this.gui.counter = this.gui.counter+1;
      if etime(this.gui.c2, this.gui.c1)>1,
        this.gui.seconds = this.gui.seconds+1;
        this.gui.framerate = this.gui.counter;
        this.gui.c1 = clock;
        this.gui.counter = 0;
      end
    end
    
    %%%% Draws all of our kinetic objects.
    function DrawSimulation(this)
      h_shapes = [];
      h_constraints = [];
      h_rayvehicles = [];
      for i = 1:numel(this.Shapes),
        h_shapes = [h_shapes, this.Shapes{i}.Draw];
      end
      for i = 1:numel(this.RayVehicles)
        h_rayvehicles = [h_rayvehicles, this.RayVehicles{i}.Draw()];
      end
      if this.gui.draw_constraint == true,
        for i = 1:numel(this.Constraints),
          h_constraints = [h_constraints, this.Constraints{i}.Draw];
        end
      end
      %If you want to access the cameraPosition, do it through the axes options
      axis equal;
      drawnow;
      
      %Now hide the objects that we don't want to see.
      if this.gui.path==false && this.gui.quit==false,
        delete(h_shapes);
        delete(h_constraints);
        delete(h_rayvehicles);
      end
    end
    
  end
  
end