classdef Compound < handle
  %COMPOUND is the superclass for all of our compounds
  
  properties
    Shapes
    Constraints
    type
    id
  end
  
  methods   
    function [Shapes] = GetShapes(this)
      Shapes = this.Shapes;
    end
  
    function [Constraints] = GetConstraints(this)
      Constraints = this.Constraints;
    end
    
    function [type] = GetType(this)
      type = this.type;
    end
    
    function [id] = GetID(this)
      id = this.id;
    end
        
    function SetPosition(this, position)
      if strcmp(this.type, 'Vehicle'),
        old_pose = this.Shapes{1}.GetPosition();
        %Move the body
        pose_diff = position-old_pose;
        this.Shapes{1}.SetPosition(position);
        %Move the wheels
        for i = 2:numel(this.Shapes)
          new_pose = this.Shapes{i}.GetPosition()+pose_diff;
          this.Shapes{i}.SetPosition(new_pose);
        end
        %Move the Hinge2 constraints, since they're the only ones that are 
        %global. 
        for i = 1:numel(this.Constraints)
          if strcmp(this.Constraints{i}.type, 'Hinge2')
            this.Constraints{i}.Anchor = this.Constraints{i}.Anchor+pose_diff;
          end
        end
      end
    end
    
    function SetID(this, id)
      this.id = id;
    end
    
  end
    
end

