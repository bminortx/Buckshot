classdef Constraint < handle
  %Constraint is a superclass that holds the the properties and methods of all 
  %constraints. It's very simple, but will help if these get more complicated. 
  
  properties (SetAccess = protected)
    type;    
    id;
  end
  
  methods (Abstract)
    %Virtual Methods
    Draw(this);
  end
  
  methods
    %%%Setters
    function SetID(this, id)
      this.id = id;
    end
    
    
    %%%Getters
    function [type] = GetType(this)
      type = this.type;
    end
    
    function [id] = GetID(this)
      id = this.id;
    end

  end
  
end

