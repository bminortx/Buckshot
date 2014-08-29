classdef SixDOF < Constraint
  %SixDOF describes a six degrees-of-freedom constraint. It's a bit more
  %complicated than going through the hinge and pivot constraints; it needs both
  %transforms (position and rotation of the shape) and bounds (how far the
  %shapes can move relative to each other, linearly and angularly). It might
  %take some thinking, but it's much more customizeable than any other
  %constraint. 
  %
  %There are two constructors to choose from, depending on how many shapes you
  %want to constrain together:
  %   1. SixDOF(Shape_A, transform_A)
  %   2. SixDOF(Shape_A, Shape_B, transform_A, transform_B)    
  %
  %There are also four limit methods, for setting lower/upper limits for
  %linear motion, and the same for angular motion. These are all 1x3 vectors
  %that hold the info for the [x,y,z] or [roll, pitch, yaw] bound.
  %SixDOF's methods for limit setting are below:
  %  
  %  *SetLinearMin(xmin, ymin, zmin) - sets the linear motion lower bound, in
  %     units proportional to the axes. 
  %  *SetLinearMax(xmax, ymax, zmax) - sets the linear motion upper bound, in
  %     radians.
  %  *SetAngularMin(pmin, qmin, rmin) - sets the angular motion lower bound, in
  %     units proportional to the axes. 
  %  *SetAngularMax(pmax, qmax, rmax) - sets the angular motion upper bound, in
  %     radians.
  %
  %Parameters: 
  % Shape_A and Shape_B: should be an object with Superclass 'Shape'.
  % transforms: 1x6 vectors giving the x, y, z, roll, pitch, and yaw of the
  %    hinge specified in local space. 
  % limits: depending on the method being set, a 1x3 vector setting the upper or
  % lower bound of either the linear or angular motion. 
  %
  
  properties
    Shape_A;
    Shape_B;
    Transform_A;
    Transform_B;
    const_in_World_A;
    const_in_World_B;
    limits;
  end
  
  methods
    %%%Constructor
    function this = SixDOF(varargin)
      if nargin == 2, 
        this.Shape_A = varargin{1};    
        transform = varargin{2};
        rpy = transform(4:6);
        quat = angle2quat(rpy(1), rpy(2), rpy(3), 'XYZ');
        this.Transform_A = [transform(1), transform(2), transform(3), quat];
        this.Shape_B = 0;
        this.limits.lin_xmax = 1;
        this.limits.lin_ymax = 1;
        this.limits.lin_zmax = 1;
        this.limits.lin_xmin = -1;
        this.limits.lin_ymin = -1;
        this.limits.lin_zmin = -1;
        this.limits.ang_pmax = 1;
        this.limits.ang_qmax = 1;
        this.limits.ang_rmax = 1;
        this.limits.ang_pmin = -1;
        this.limits.ang_qmin = -1;
        this.limits.ang_rmin = -1;
        this.type = 'SixDOF_one';
        position_A = this.Shape_A.GetPosition();
        this.const_in_World_A = position_A+this.Transform_A(1:3);
      elseif nargin == 4, 
        this.Shape_A = varargin{1};
        this.Shape_B = varargin{2};
        transform = varargin{3};
        rpy = transform(4:6);
        quat = angle2quat(rpy(1), rpy(2), rpy(3), 'XYZ');
        this.Transform_A = [transform(1), transform(2), transform(3), quat];
        transform = varargin{4};
        rpy = transform(4:6);
        quat = angle2quat(rpy(1), rpy(2), rpy(3), 'XYZ');
        this.Transform_B = [transform(1), transform(2), transform(3), quat];
        this.limits.lin_xmax = 1;
        this.limits.lin_ymax = 1;
        this.limits.lin_zmax = 1;
        this.limits.lin_xmin = -1;
        this.limits.lin_ymin = -1;
        this.limits.lin_zmin = -1;
        this.limits.ang_pmax = 1;
        this.limits.ang_qmax = 1;
        this.limits.ang_rmax = 1;
        this.limits.ang_pmin = -1;
        this.limits.ang_qmin = -1;
        this.limits.ang_rmin = -1;
        this.type = 'SixDOF_two';
      else 
        disp('Unknown arguments for SixDOF. Check the help');
      end
      this.id = 0;
    end
    
    %%%Methods
    function [h] = Draw(this)
      h = [];
      if strcmp(this.type, 'SixDOF_one'),
        h = scatter3(this.const_in_World_A(1),...
          this.const_in_World_A(2),...
          this.const_in_World_A(3),...
          'MarkerEdgeColor', 'k', ...
          'MarkerFaceColor', 'b');
      elseif strcmp(this.type, 'SixDOF_two'),
        %I got nothing.
      end
      
    end
       
    
    
    %%%Setters
    function SetLinearMax(this, xmax, ymax, zmax)
      this.limits.lin_xmax = xmax;
      this.limits.lin_ymax = ymax;
      this.limits.lin_zmax = zmax;
    end 
     
    function SetLinearMin(this, xmin, ymin, zmin)
      this.limits.lin_xmin = xmin;
      this.limits.lin_ymin = ymin;
      this.limits.lin_zmin = zmin;
    end 
    
    function SetAngularMax(this, pmax, qmax, rmax)
      this.limits.ang_pmax = pmax;
      this.limits.ang_qmax = qmax;
      this.limits.ang_rmax = rmax;
    end
    
    function SetAngularMin(this, pmin, qmin, rmin)
      this.limits.ang_pmin = pmin;
      this.limits.ang_qmin = qmin;
      this.limits.ang_rmin = rmin;
    end
    
    
    %%%Getters
    function [limits] = GetLimits(this)
      limits = [this.limits.lin_xmax,...
        this.limits.lin_ymax,...
        this.limits.lin_zmax,...
        this.limits.lin_xmin,...
        this.limits.lin_ymin,...
        this.limits.lin_zmin,...
        this.limits.ang_pmax,...
        this.limits.ang_qmax,...
        this.limits.ang_rmax,...
        this.limits.ang_pmin,...
        this.limits.ang_qmin,...
        this.limits.ang_rmin];
    end
    
    function [Shape_A] = GetShape_A(this)
      Shape_A = this.Shape_A;
    end
    
    function [Shape_B] = GetShape_B(this)
      Shape_B = this.Shape_B;
    end
    
    function [transform_A] = GetTransform_A(this)
      transform_A = this.Transform_A;
    end
    
    function [transform_B] = GetTransform_B(this)
      transform_B = this.Transform_B;
    end
    
  end
  
end



















