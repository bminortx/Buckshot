classdef Hinge2 < Constraint
  %Hinge2 activates a Hinge2 constraint. From bullet documentation: 
  %  >"Constraint similar to ODE Hinge2 Joint
  %  > Has 3 degrees of freedom:
  %  > 2 rotational degrees of freedom, similar to Euler rotations around 
  %  >    Z (axis 1) and X (axis 2).
  %  > 1 translational (along axis Z) with suspension spring"
  %
  %There is only one constructor for the Hinge2 constraint: 
  %   Hinge2(Shape_A, Shape_B, Anchor, Axis_1, Axis_2) 
  %
  %Unlike other constraints, the anchor and axes vectors are positioned in the
  %world coordinate system. Axis_1 must be orthogonal to Axis_2. As the bullet
  %documentation states, Axis_1 usually describes the Z+ axis, and Axis_2
  %describes the X axis. As this hinge is actually a subclass of the generic
  %6DOF constraint for springs, it has methods to set stiffness and damping in
  %the translational movement of Axis_1 (the 'suspension', if you will). 
  %
  %There are four methods that can be used to set the limits on each axis:
  %  SetStiffness(numStiffness): sets the stiffness of the suspension in Axis_1.
  %    10 - Offroad buggy
  %    50 - Sports Car
  %    200 - F1 Car
  %  SetDamping(dampFactor): sets the damping factor for the hinge. numDamping 
  %    should be a number between 0 and 1. Damping is set according to the 
  %    stiffness provided:
  %      damping = dampFactor * 2.0 * sqrt(stiffness)
  %    dampFactor between .1 and .2 is usually recommended. 
  %  SetMinRotation(rotAngle): Pass a 1x3 vector describing the minimum
  %    angular motion for Axis_1, or the turn in the wheel. 
  %  SetMaxRotation(rotAngle): Pass a 1x3 vector describing the maximum
  %    angular motion for Axis_1, or the turn in the wheel. 
  %
  %Parameters:
  %  Shape_A and Shape_B: should be an object with Superclass 'Shape'.
  %  Anchor: 1x3 vector specifying the x, y, and z position of the location of 
  %    the crosspoint of the two axes.  
  %  axes: 1x3 vectors specifying the x, y, and z axis in world space. 
  %
  %
  %Sources to check out for more info:
  %  *http://www.ode.org/ode-latest-userguide.html - picture of Hinge2.
  %  *Kester Maddock's tutorial "Vehicle Simulation with Bullet". 
  
  properties
    Shape_A;
    Shape_B;
    Anchor;
    Axis_1;
    Axis_2;
    dampFactor;
    damping;
    stiffness;
    steering_angle;
    min_rotation;
    max_rotation;
  end
  
  methods
    %%%Constructor
    function this = Hinge2(Shape_A, Shape_B, Anchor, Axis_1, Axis_2)
      if dot(Axis_1, Axis_2)~=0, 
        disp('Axis_1 and Axis_2 must be perpendicular to each other.');
        return;
      end
      this.Shape_A = Shape_A;
      this.Shape_B = Shape_B;
      this.Anchor = Anchor;
      this.Axis_1 = Axis_1;
      this.Axis_2 = Axis_2;
      this.dampFactor = .2;
      this.stiffness = 50;
      this.min_rotation = -pi/6;
      this.max_rotation = pi/6;
      this.InterpDamping();
      this.type = 'Hinge2';
      this.id = 0;
      this.steering_angle = 0;
    end
    
    %%%Methods
    function [h] = Draw(this)
      h = scatter3(this.Anchor(1),...
        this.Anchor(2),...
        this.Anchor(3),...
        'MarkerEdgeColor', 'k', ...
        'MarkerFaceColor', 'k');
    end
    
    function InterpDamping(this)
      this.damping = this.dampFactor * 2 * sqrt(this.stiffness);
    end
    
    %%%Setters 
    function SetSteeringAngle(this, angle)
      this.steering_angle = angle;
    end
    
    function SetDamping(this, dampFactor)
      this.dampFactor = dampFactor;
      this.InterpDamping;
    end
    
    function SetStiffness(this, stiffness)
      this.stiffness = stiffness;
      this.InterpDamping;
    end
    
    function SetMinRotation(this, rotAngle)
      this.min_rotation = rotAngle;
    end
    
    function SetMaxRotation(this, rotAngle)
      this.max_rotation = rotAngle;
    end
    
    function SetPosition(this, position)
      this.Anchor = position;
    end
    
    %%%Getters
    function [damping] = GetDamping(this)
      damping = this.damping;
    end
    
    function [stiffness] = GetStiffness(this)
      stiffness = this.stiffness;
    end
    
    function [min_rotation] = GetMinRotation(this)
      min_rotation = this.min_rotation;
    end
    
    function [max_rotation] = GetMaxRotation(this)
      max_rotation = this.max_rotation;
    end
    
    function [steering_angle] = GetSteeringAngle(this)
      steering_angle = this.steering_angle;
    end
    
  end
  
end

