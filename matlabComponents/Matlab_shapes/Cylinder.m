classdef Cylinder < Shape
  %A cylinder class for our Shapes
  
  properties
    radius; 
    height;
    x_vals;
    y_vals;
    z_vals;
  end
  
  methods
    %%%Constructor
    function this = Cylinder(radius, height, mass, restitution)
      this.radius = radius;
      this.height = height;
      this.mass = mass;
      this.type = 'Cylinder';
      this.restitution = restitution;
      this.position = [0,0,0];
      this.rotation = [1 0 0;
        0 1 0;
        0 0 1];
      this.id = 0;
      this.color = [.5, .5, .5];
      z=[-(this.height/2);(this.height/2)];
      angle=linspace(0,2*pi,40);
      [z,angle]=meshgrid(z,angle);
      x=this.radius*cos(angle);
      y=this.radius*sin(angle);
      this.x_vals = x;
      this.y_vals = y;
      this.z_vals = z;
    end
  
    %%%Methods
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
    
    %%%Setters
    
    function SetRadius(this, radius)
      this.radius = radius;
    end
    
    function SetHeight(this, height)
      this.height = height;
    end
    
    %%%Getters
    
    function [radius] = GetRadius(this)
      radius = this.radius;
    end
    
    function [height] = GetHeight(this)
      height = this.height;
    end
    
  end
    
  
end

