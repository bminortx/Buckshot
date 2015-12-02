classdef PointToPoint < Constraint
  %Properties and methods for a point-to-point constraint, aka a ball and socket
  %constraint. All vectors are in LOCAL space. 
  %
  %There are two constructors for a PointToPoint object: 
  % 1. PointToPoint(Shape_A, pivot_in_A)
  %      This is a constraint for a shape to empty space. 
  % 2. PointToPoint(Shape_A, Shape_B, pivot_in_A, pivot_in_B)
  %      This is a constraint between two shapes. 
  
  
  properties (SetAccess = public)
    Shape_A; 
    Shape_B;
    pivot_in_A;
    pivot_in_B;
    pivot_in_World_A;
    pivot_in_World_B;
  end
  
  methods
    %%%constructor - two shapes
    function this = PointToPoint(varargin)
      if nargin== 4,
        this.Shape_A = varargin{1};
        this.Shape_B = varargin{2};
        this.pivot_in_A = varargin{3};
        this.pivot_in_B = varargin{4};
        this.type = 'PointToPoint_two';
      elseif nargin == 2, 
        this.Shape_A = varargin{1};
        this.pivot_in_A = varargin{2};
        this.Shape_B = 0;
        this.pivot_in_B = 0;
        position_A = this.Shape_A.GetPosition();
        this.pivot_in_World_A = position_A+this.pivot_in_A;
        this.type = 'PointToPoint_one'; 
      else
        disp('Unexpected parameters. Check the help file for constructors');
        exit;
      end
      this.id = 0;
    end
    
    %%%Methods
    function [h] = Draw(this)
      if strcmp(this.type, 'PointToPoint_one'), 
        h = scatter3(this.pivot_in_World_A(1),...
          this.pivot_in_World_A(2),...
          this.pivot_in_World_A(3),...
          'MarkerEdgeColor', 'k', ...
          'MarkerFaceColor', 'g');
      else
        position_A = this.Shape_A.GetPosition();
        rotation_A = this.Shape_A.GetRotation();
        this.pivot_in_World_A = (rotation_A*this.pivot_in_A')'+position_A;
        position_B = this.Shape_B.GetPosition();
        rotation_B = this.Shape_B.GetRotation();
        this.pivot_in_World_B = (rotation_B*this.pivot_in_B')'+position_B;
        this.pivot_in_World_B = position_B+this.pivot_in_B;
        pivots = [this.pivot_in_World_A; this.pivot_in_World_B];
        h = scatter3(pivots(:,1),...
          pivots(:,2),...
          pivots(:,3),...
          'MarkerEdgeColor', 'k', ...
          'MarkerFaceColor', 'g');
      end
    end
    
    
    
    %%%setters
    function Set_Shape_A(this, Shape)
      this.Shape_A = Shape;
    end
  
    function Set_Shape_B(this, Shape)
      this.Shape_B = Shape;
    end
    
    function Set_pivot_in_A(this, pivot)
      this.pivot_in_A = pivot;
    end
    
    function Set_pivot_in_B(this, pivot)
      this.pivot_in_B = pivot;
    end
    
  end
  
end
