classdef Terrain < handle
  %A heightmap class to use in MATLAB
  %Constructor: 
  %  Terrain(width, length, max_ht, granularity)
  
  properties (SetAccess = private)
    x_vals; %1xdomain
    y_vals; %1xrange
    z_vals; %domainxrange
    id;
    granularity;
    normal;
    %Not pretty, but it works for now.
    height_fun;
    medx;
    medy;
    medz;
    x_vals_2d;
    y_vals_2d;
    z_vals_2d;
  end
  
  methods
    %constructor
    function this = Terrain(width, length, max_ht, granularity)
      if max_ht<=1,
        %Make all heights = 0
        x = rand(100, 1)*width;
        y = rand(100, 1)*length;
        z = rand(100, 1)*0;
        xlin = linspace(min(x), max(x), width*granularity);
        ylin = linspace(min(y), max(y), length*granularity);
        [X,Y]=meshgrid(xlin,ylin);
        f = scatteredInterpolant(x, y, z);
        Z = f(X,Y);
      else
        x = rand(100, 1)*width;
        y = rand(100, 1)*length;
        %These commands center our plot.
        z = rand(100, 1)*max_ht;
        xlin = linspace(min(x), max(x), width*granularity);
        ylin = linspace(min(y), max(y), length*granularity);
        [X,Y]=meshgrid(xlin,ylin);
        f = scatteredInterpolant(x, y, z);
        Z = f(X,Y);
      end
      this.medx = (max(max(X)) + min(min(X)))/2;
      this.medy = (max(max(Y)) + min(min(Y)))/2;
      this.x_vals = X-repmat(this.medx, size(X));
      this.y_vals = Y-repmat(this.medy, size(Y));
      this.medz = (max(max(Z)) + min(min(Z)))/2; 
      this.z_vals = Z-repmat(this.medz, size(Z));
      this.granularity = granularity;
      this.id = 0;
      this.normal = [0,0,1];
      this.height_fun = f;
      this.x_vals_2d = [];
      this.y_vals_2d = [];
      this.z_vals_2d = [];
    end
    
    %%%%%%%%%%%%%%%%%%%%
    
    %methods
    function Draw(this)
      mesh(this.x_vals, this.y_vals, this.z_vals);
      colormap('summer');
      axis equal;
      if isempty(this.x_vals_2d)==0, 
        plot3(this.x_vals_2d, this.y_vals_2d, this.z_vals_2d, 'b.-', ...
          'MarkerSize', 10);
        plot3(this.x_vals_2d(end), this.y_vals_2d(end), this.z_vals_2d(end), 'g*', ...
          'MarkerSize', 30);
      end
    end
    
    function DrawContour(this, boolDrawLabel)
      %A handy method to see the contour map the mesh produced.
      [C,h]=contour(this.x_vals,this.y_vals,this.z_vals);
      if boolDrawLabel==true, 
        clabel(C,h,'FontSize', 10, 'Color', 'r', 'Rotation', 0);
      end
    end
    
    function Add2DPath(this, x_poses, y_poses)
      % path is a set of (x, y) points that outline a path, usually one returned
      % from SplineOptimize.
      if iscell(x_poses)==1, 
        for i=1:numel(x_poses),
          sub_x = x_poses{i};
          sub_y = y_poses{i};
          sub_z = zeros(1, numel(sub_x));
          for j=1:numel(sub_x), 
            sub_z(j) = 10;
%             this.height_fun(sub_x(j), sub_y(j))-this.medz
          end
          this.x_vals_2d = [this.x_vals_2d sub_x];
          this.y_vals_2d = [this.y_vals_2d sub_y];
          this.z_vals_2d = [this.z_vals_2d sub_z];
        end
      else
        z_poses = zeros(1, numel(x_poses));
        for j=1:numel(x_poses),
          z_poses(j) = 10;
        end
        this.x_vals_2d = [this.x_vals_2d x_poses];
        this.y_vals_2d = [this.y_vals_2d y_poses];
        this.z_vals_2d = [this.z_vals_2d z_poses];
      end
    end
    
    
    %%%%%%%%%%%%%%%%%%%%
    
    %There are no setters, since the heightmap should not be changed in
    %simulation. 
    function SetID(this, id)
      this.id = id;
    end
    
    function SetPosition(this, position)
      this.z_vals = this.z_vals-repmat(position(3), size(this.z_vals));
    end
    
    function SetRotation(this, rotation)
      this.rotation = rotation;
    end
    
    function SetNormal(this, normal)
      this.normal = normal;
    end
    
    %%%%%%%%%%%%%%%%%%%%
    
    %getters
    function [Dim] = GetDim(this)
      min_x = min(min(this.x_vals));
      min_y = min(min(this.y_vals));
      min_z = min(min(this.z_vals));
      max_x = max(max(this.x_vals));
      max_y = max(max(this.y_vals));
      max_z = max(max(this.z_vals));
      Dim = [min_x max_x; 
             min_y max_y;
             min_z max_z];
    end
    
    function [Ranges] = GetRanges(this)
      range_x = max(max(this.x_vals)) - min(min(this.x_vals));
      range_y = max(max(this.y_vals)) - min(min(this.y_vals));
      range_z = max(max(this.z_vals)) - min(min(this.z_vals));
      Ranges = [range_x, range_y, range_z];
    end
    
    function [granularity] = GetGranularity(this)
      granularity  = this.granularity;
    end
    
    function [heightmap] = GetHeightmap(this)
      heightmap = {this.x_vals this.y_vals this.z_vals};
    end
    
    function [id] = GetID(this)
      id = this.id;
    end
    
    function [normal] = GetNormal(this)
      normal = this.normal;
    end
  end
  
end

