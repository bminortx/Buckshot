classdef Shape < handle
  %The abstract class that is used in MATLAB
  
  properties (SetAccess = protected)
    mass;
    restitution; 
    position;
    rotation; 
    id;
    color;
    type;
  end
  
  methods (Abstract)
    %Virtual Methods
    Draw(this);   
  end
  
  methods
    
    %getters
    function [position] = GetPosition(this)
      position = this.position;
    end
    
    function [rotation] = GetRotation(this)
      rotation = this.rotation;
    end
    
    function [mass] = GetMass(this)
      mass = this.mass;
    end
    
    function [restitution] = GetRestitution(this)
      restitution = this.restitution;
    end
    
    function [id] = GetID(this)
      id = this.id;
    end
    
    function [color] = GetColor(this)
      color = this.color;
    end
    
    function [type] = GetType(this)
      type = this.type;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %setters
    function SetPosition(this, position)
      this.position = position;
    end
    
    function SetRotation(this, rotation)
      %We always want a 3x3 rotation matrix.
      [m,n] = size(rotation);
      if m == 1 && n == 3, 
        rotquat = angle2quat(rotation(1), rotation(2), rotation(3), 'XYZ');
        rotation = quat2rot(rotquat);
      end
      if m == 1 && n == 4, 
        rotation = quat2rot(rotquat);
      end
      this.rotation = rotation;
    end
    
    function SetTransform(this, position, rotation)
      this.position = position;
      this.rotation = rotation;
    end
    
    function SetMass(this, mass)
       this.mass = mass;
    end
    
    function SetRestitution(this, restitution)
      this.restitution = restitution;
    end
    
    function SetID(this, id)
      this.id = id;
    end
    
    function SetColor(this, color)
      this.color = color;
    end
    
  end
  
end

