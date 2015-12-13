classdef Hinge < Constraint
  %A Hinge (good for wheels); there is a aligned and non-aligned constructor.
  %Since a hinge is a bit more customizeable, it has more setters than usual.
  %All vectors and transforms are in LOCAL space. 
  %
  %There are four constructors for a Hinge object: 
  %  1. Hinge(Shape_A, transform_A)
  %       Creates a hinge in space on Shape A with hinge transform A.
  %  2. Hinge(Shape_A, Shape_B, transform_A, transform_B)
  %       Creates an aligned hinge between A and B, with hinge placements  
  %       transformed by their relative t_. Play around to better understand...
  %  3. Hinge(Shape_A, pivot_in_A, axis_in_A)
  %       Creates a hinge on Shape A through the pivot point and axis provided. 
  %  4. Hinge(Shape_A, Shape_B, pivot_in_A, pivot_in_B, 
  %           axis_in_A, axis_in_B)
  %       Hinges both A and B according to parameters given. 
  %
  %Hinge's method SetLimits allows the user to set the following values: 
  %  *The lowest angle allowed for the hinge, in radians
  %  *The highest angle allowed for the hinge, in radians
  %  *The softness: the factor at which the velocity error correction 
  %       starts operating, i.e a softness of 0.9 means that the vel. corr 
  %       starts at 90% of the limit range.
  %  *The bias factor: the magnitude of the position correction. It tells you 
  %       how strictly the position error (drift) is corrected.
  %  *The relaxation factor:  the rate at which velocity errors are corrected.
  %       This can be seen as the strength of the limits. A low value will 
  %       make the the limits more spongy.
  %
  %
  %Parameters: 
  %  Shape_A and Shape_B: should be an object with Superclass 'Shape'.
  %  pivots: 1x3 vectors specifying the x, y, and z position in local space,
  %    relative to the shape it is attributed to.  
  %  axes: 1x3 vectors specifying the x, y, and z axis in local space,
  %    relative to the shape it is attributed to.  
  %  transforms: 1x6 vectors giving the x, y, z, roll, pitch, and yaw of the
  %    hinge specified in local space. 
  %  limits: a 1x5 vector specifying: 
  %    [low, high, softness, bias factor, relaxation factor]    
  %    Default values: [-1, 1, .9, .3, 1]
  %
  
  properties (SetAccess = public)
    Shape_A;
    Shape_B;
    transform_A;
    transform_B;
    pivot_in_A;
    pivot_in_B;
    pivot_in_World_A;
    pivot_in_World_B;
    axis_in_A;
    axis_in_B;
    axis_in_World_A;
    axis_in_World_B;
    rot_values_A;
    rot_values_B;
    limits;
  end
  
  methods
    %%%Constructors
    function this = Hinge(varargin)
      this.limits.low = -1;
      this.limits.high = 1;
      this.limits.softness = .9;
      this.limits.bias_factor = .3;
      this.limits.relaxation_factor = 1;
      this.id = 0;
      if nargin == 2,
        this.Shape_A = varargin{1};
        transform = varargin{2};
        rpy = transform(4:6);
        quat = angle2quat(rpy(1), rpy(2), rpy(3), 'XYZ');
        this.Transform_A = [transform(1), transform(2), transform(3), quat];
        this.pivot_in_A = 0;
        this.Shape_B = 0;
        this.transform_B = 0;
        this.type = 'Hinge_one_transform';
      elseif nargin == 3,
        this.Shape_A = varargin{1};
        this.pivot_in_A = varargin{2};
        this.axis_in_A = varargin{3};
        position_A = this.Shape_A.GetPosition();
        this.pivot_in_World_A = this.pivot_in_A+position_A;
        this.axis_in_World_A = [5*this.axis_in_A; -5*this.axis_in_A];
        this.axis_in_World_A = this.axis_in_World_A + ...
                               repmat(this.pivot_in_World_A, 2, 1);
        this.Shape_B = 0;
        this.transform_A = 0;
        this.type = 'Hinge_one_pivot';
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
        this.pivot_in_A = 0;
        this.type = 'Hinge_two_transform';
      elseif nargin == 6,
        this.Shape_A = varargin{1};
        this.Shape_B = varargin{2};
        this.pivot_in_A = varargin{3};
        this.pivot_in_B = varargin{4};
        this.axis_in_A = varargin{5};
        this.axis_in_B = varargin{6};
        this.transform_A = 0;
        this.type = 'Hinge_two_pivot';
      end
    end
    
    
    %%%Methods
    function [h] = Draw(this)
      h = [];
      if strcmp(this.type,'Hinge_one_transform'), 
        %%%I got nothing for this right now...
        
      elseif strcmp(this.type,'Hinge_one_pivot'),
        h = [h, scatter3(this.pivot_in_World_A(1),...
          this.pivot_in_World_A(2),...
          this.pivot_in_World_A(3),...
          'MarkerEdgeColor', 'k', ...
          'MarkerFaceColor', 'r')];
        h = [h, line(this.axis_in_World_A(:, 1),...
          this.axis_in_World_A(:, 2),...
          this.axis_in_World_A(:, 3))];
        %Draw the arc...? 
%         h = [h, line(this.rot_values_A(:,1),...
%           this.rot_values_A(:,2),...
%           this.rot_values_A(:,3))];
%         
      elseif strcmp(this.type,'Hinge_two_transform'), 
        %%% Again, nothing...
        
      elseif strcmp(this.type,'Hinge_two_pivot'), 
        position_A = this.Shape_A.GetPosition();
        position_B = this.Shape_B.GetPosition();
        this.pivot_in_World_A = this.pivot_in_A+position_A;
        this.pivot_in_World_B = this.pivot_in_B+position_B;
        pivots = [this.pivot_in_World_A; this.pivot_in_World_B];
        this.axis_in_World_A = [5*this.axis_in_A; -5*this.axis_in_A];
        this.axis_in_World_B = [5*this.axis_in_B; -5*this.axis_in_B];
        this.axis_in_World_A = this.axis_in_World_A + ...
                               repmat(this.pivot_in_World_A, 2, 1);
        this.axis_in_World_B = this.axis_in_World_B + ...
                               repmat(this.pivot_in_World_B, 2, 1);
        hold on;
        h = [h, scatter3(pivots(:,1),...
          this.pivot_in_World_A(:,2),...
          this.pivot_in_World_A(:,3),...
          'MarkerEdgeColor', 'k', ...
          'MarkerFaceColor', 'r')];
        h = [h, line(this.axis_in_World_A(:, 1),...
          this.axis_in_World_A(:, 2),...
          this.axis_in_World_A(:, 3),...
          'r')];
        h = [h, line(this.axis_in_World_B(:, 1),...
          this.axis_in_World_B(:, 2),...
          this.axis_in_World_B(:, 3),...
          'r')];
        
      end
      
    end
        
    %Ony applies to certain constraints (those with transforms). Turns a
    %roll-pitch-yaw sequence into a useable quaternion. 
    function [transform] = CreateTransform(transform)
      rpy = transform(4:6);
      quat = angle2quat(rpy, 'XYZ');
      transform = [transform(1:3), quat];
    end
    
    
    
    
    %%%Setters
    function SetLimits(this, varargin)
      if nargin == 3,
        this.limits.low = varargin{1};
        this.limits.high = varargin{2};
      elseif nargin == 6,
        this.limits.low = varargin{1};
        this.limits.high = varargin{2};
        this.limits.softness = varargin{3};
        this.limits.bias_factor = varargin{4};
        this.limits.relaxation_factor = varargin{5};
      else 
        disp('There is something wrong with your limits...');
      end
    end
    
    %%%Getters
    
    function [limits] = GetLimits(this)
      limits = [this.limits.low, this.limits.high, this.limits.softness, ...
                this.limits.bias_factor, this.limits.relaxation_factor];
    end
    
  end
  
end

