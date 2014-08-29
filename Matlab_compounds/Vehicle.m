classdef Vehicle < Compound
  % A vehicle made of Shapes and Constraints. There is only one constructor
  % associated with it:
  %
  %  Vehicle(params)
  %
  % This 'params' structure is pretty complicated; it has all of the physical
  % information that the program needs to build the car. LoadVehicleParams
  % populates this struct for you automatically, allowing you to modify
  % essential parameters there.
  %
  % The structure of the car itself is pretty simple. There is a cube shape as
  % the body, and four cylinders constrained to the cube's 'axle'. The car goes
  % forward by putting torque on the back two wheels and turns by rotating the
  % front wheels relative to the up axis. The suspension on the wheels is
  % dictated by the softness and bias of the hinge constraint that is assigned to
  % each wheel. 
  %
  % Relative axes: 
  %    * x+ is right
  %    * y+ is forward
  %    * z+ is up
  %
  % Being a compound shape, there are both Constraints and Shapes to add to the
  % BulletSim when construction is complete. The method for that is in
  % bullet_interface.m. 
  
  properties
    params;
    state;
    body;
    wheel;
  end
  
  methods
    %%%Constructor
    function this = Vehicle()
      this.type = 'Vehicle';
      this.id = 0;
      LoadVehicleParams(this);

      %%%%%%%%%%%%%%%%%
      %%%Create the car shapes, and move them into position. 
      %%%%%%%%%%%%%%%%%
      this.Shapes{1} = Cube(this.body.width, this.body.length, ...
                             this.body.height, this.body.mass, ...
                             this.body.restitution);
      this.Shapes{1}.SetPosition([0, 0, this.body.base_ht+...
                                          (this.body.height/2)]);

      for i = 1:4,
        this.Shapes{i+1} = Cylinder(this.wheel{i}.radius, ...
                                        this.wheel{i}.width,...
                                        this.wheel{i}.mass,...
                                        this.wheel{i}.restitution);
        this.Shapes{i+1}.SetRotation([0, pi/2, 0]);
      end
      axle_width = this.body.width*.5+this.wheel{1}.width*.5+.1;
      axle_length = this.body.length*.5-this.body.length*.1;
      axle_height = this.wheel{1}.radius;
      this.Shapes{2}.SetPosition([-axle_width,  axle_length, axle_height]);
      this.Shapes{3}.SetPosition([ axle_width,  axle_length, axle_height]);
      this.Shapes{4}.SetPosition([-axle_width, -axle_length, axle_height]);
      this.Shapes{5}.SetPosition([ axle_width, -axle_length, axle_height]);
      
      %%%%%%%%%%%%%%%%%
      %%%Create the car constraints.
      %%%%%%%%%%%%%%%%%
      anchor_width = this.body.width/4;
      anchor_length = axle_length;
      anchor_ht = axle_height;
      %Front left
      Anchor_fl = [-anchor_width,  anchor_length, anchor_ht];
      Axis_fl = [1, 0, 0];
      %Front right
      Anchor_fr = [ anchor_width,  anchor_length, anchor_ht];
      Axis_fr = [1, 0, 0];
      %Back left
      Anchor_bl = [-anchor_width, -anchor_length, anchor_ht];
      Axis_bl = [1, 0, 0];
      %Back right
      Anchor_br = [ anchor_width, -anchor_length, anchor_ht];
      Axis_br = [ 1, 0, 0];
      %Z-axis for all of the tires' hinge2 constraints, ie the suspension.
      Susp_Axis = [0, 0, 1];

      %Add the constraints.
      this.Constraints{1} = Hinge2(this.Shapes{1}, this.Shapes{2},...
        Anchor_fl, Susp_Axis, Axis_fl);
      this.Constraints{2} = Hinge2(this.Shapes{1}, this.Shapes{3},...
        Anchor_fr, Susp_Axis, Axis_fr);
      this.Constraints{3} = Hinge2(this.Shapes{1}, this.Shapes{4},...
        Anchor_bl, Susp_Axis, Axis_bl);
      this.Constraints{4} = Hinge2(this.Shapes{1}, this.Shapes{5},...
        Anchor_br, Susp_Axis, Axis_br);
      for i = 1:4, 
        this.Constraints{i}.SetDamping(this.params.dampFactor);
        this.Constraints{i}.SetStiffness(this.params.stiffness);
      end 
      this.Constraints{1}.SetSteeringAngle(this.state.steering_angle);
      this.Constraints{2}.SetSteeringAngle(this.state.steering_angle);
      this.Constraints{3}.SetSteeringAngle(0);
      this.Constraints{4}.SetSteeringAngle(0);
    end
    
    %%%Methods
    
    
    
    
    %%%Setters 
    
    function SetSteering(this, steering)
      this.state.steering_angle = steering;
    end
    
    function SetForce(this, force)
      this.state.force = force;
    end
    
    
    
    %%%Getters 
    
    function steering = GetSteering(this)
      steering = this.state.steering_angle;
    end
    
    function force = GetForce(this)
      force = this.state.force;
    end
    
    
  end
  
end

