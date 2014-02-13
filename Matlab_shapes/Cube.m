classdef Cube < Shape
  %The cube class, for use in MATLAB
  
  properties (SetAccess = private)
    %class attributes
    width;
    length;
    height;
    
  end
  
  methods
    %Constructor
    function this = Cube(width, length, height, mass, restitution)
      this.width = width;
      this.length = length;
      this.height = height;
      this.mass = mass;
      this.type = 'Cube';
      this.restitution = restitution;
      this.position = [0,0,0];
      this.rotation = [1,0,0; 
                       0,1,0; 
                       0,0,1];
      this.id = 0;
      this.color = [0, 0, 1];
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Methods
    function hpatch = Draw(this)
      w = this.width/2;
      l = this.length/2;
      h = this.height/2;
      rotmat = this.rotation;
      points = [ w w -w -w w w -w -w; 
                 l -l -l l l -l -l l;
                 h h h h -h -h -h -h];
      points = (rotmat*points)+repmat(this.position', 1, numel(points(1,:)));
      %Now plot.
      vertices = points';
      faces = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
      hpatch = patch('Vertices', vertices, 'Faces', faces, 'FaceColor', this.color);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    %Getters
    function [Dim] =  GetDim(this)
      Dim = [this.width; this.length; this.height];
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %Setters
    function SetDim(this, width, length, height)
      this.width = width;
      this.length = length;
      this.height = height;
    end
    
  end
  
end

