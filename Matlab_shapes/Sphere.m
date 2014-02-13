classdef Sphere < Shape
  %A sphere class, for use in MATLAB
  
  properties (SetAccess = private)
    %class attributes
    radius;
    x_vals;
    y_vals;
    z_vals;
    
  end
  
  methods
    %constructor
    function this = Sphere(radius, mass, restitution)
      this.radius = radius;
      this.mass = mass; 
      this.type = 'Sphere';
      this.restitution = restitution; 
      this.position = [0,0,0];
      this.rotation = [1 0 0; 
                       0 1 0; 
                       0 0 1];
      this.id = 0;
      this.color = [1,0,0];
      [oldx,oldy,oldz] = sphere;
      x = [];
      y = [];
      z = [];
      oldx = this.radius*oldx;
      oldy = this.radius*oldy;
      oldz = this.radius*oldz;
      for i=1:numel(oldx(:,1)),
        if mod(i, 2)==1,
          x = [x; oldx(i, :)];
          y = [y; oldy(i, :)];
          z = [z; oldz(i, :)];
        end
      end
      this.x_vals = x;
      this.y_vals = y;
      this.z_vals = z;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %methods    
    function hsurf = Draw(this) 
      x = this.x_vals;
      szx = size(x);
      y = this.y_vals;
      z = this.z_vals;
      x = reshape(x, 1, numel(x));
      y = reshape(y, 1, numel(y));
      z = reshape(z, 1, numel(z));
      point = [x; y; z];
      rotmat = this.rotation;
      point = (rotmat*point)+repmat(this.position', 1, numel(x(1,:)));
      x = reshape(point(1,:), szx);
      y = reshape(point(2,:), szx);
      z = reshape(point(3,:), szx);
      hsurf = surf(x,y,z, 'FaceColor', this.color);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %getters
    
    function [radius] = GetRadius(this)
      radius = this.radius;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %setters
    
    function SetRadius(this, radius)
      this.radius = radius;
    end
    
  end
  
end

