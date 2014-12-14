function [x_poses, y_poses, th_poses, major_poses] = SplineOptimize(Map, poses, gui_on)
% SplineOptimize uses splines and Gauss-Newton minimization to find the path in
% 3D space with the lowest cost. There are two different ways that 
% SplineOptimize can be used:
%
% 1. SplineOptimize(Matlab_shape::Terrain, startpose, endpose, gui_on): 
%    given a startpose and an endpose, and the map that it should optimize over, 
%    SplineOptimize will discretize the straight line between the goals into a 
%    series of minor points and optimize this path. gui_on is a bool that tells
%    the function whether or not to show the user the optimization.
%
% 2. SplineOptimize(gui_on): runs the same process as (1), but uses random data. 
%    
% If nothing is passed in, the function just assumes you wanna see the gui.
%
% SplineOptimize returns five matrices:
%  - x_poses: the x coordinate of every point
%  - y_poses: the y coordinate of every point
%  - th_poses: the theta of every point
%  - major_poses: the ends of our splines

% Splines are calculated using MATLAB's 'spline' function, which creates a cubic
% spline from the two endpoints and slopes given. 
%
% This should be enough information to pass to a physics-driven vehicle
% simulator like Matlab_compounds::RaycastVehicle.

disp('Starting to optimize our path!');

%%%%%%%%%%%%
%%% DETERMINE OUR ARGUMENTS
%%% n is the number of spline endpoints on our map; thus, we have n-1 splines
%%%%%%%%%%%%

startpose = poses(:,1);
endpose = poses(:,end);
if numel(startpose)~=4 && numel(endpose)~=4,
  disp('One of your vector arguments has been input incorrectly.');
  disp('Vectors = [x, y, theta, velocity].');
  return;
end
distance = norm( abs( [endpose(2)-startpose(2), endpose(1)-startpose(1)] ), 2 );
n = numel( poses(1, :) );

%%%%%%%%%%%%
%%% GUI COMMANDS
%%%%%%%%%%%%
if gui_on,
  close all;
  hold on;
  grid on;
  axis equal;
  xlabel('x');
  ylabel('y');
  h = [];                           % The handle matrix for the graphics
end

%%%%%%%%%%%%
%%% DATA STRUCTURES
%%%%%%%%%%%%

deltaT = 3;                       % The time taken between points 
t = 1:deltaT;
maxIter = 2000;                   % maximum iterations of our GN. 
originposes = zeros(4, n);        % holds the original point coordinates
hit_min = 0;                      % GN cuts off when it doesn't minimize 
                                     % again after so many iterations
vars = 3;                         % Dimensions we are minimizing
old_dist = Inf;                   % Holds the distance of the last path
prev_norm_Deltas = Inf;           % Holds previous norm deltas

major_poses = [];                 % our final poses [ x ; y ]
x_poses = {};
y_poses = {};
th_poses = {};
x_f = [];
y_f = [];

c1 = clock;
c2 = 0;

%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%
for jj=0:maxIter,
  c2 = clock;
  %%%%%%%%%%%%
  %%% RESET OUR MATRICES
  %%%%%%%%%%%%
  distances = zeros(n-1, 1);
  listx = [];
  listy = [];
  
  %%%%%%%%%%%%
  %%% PLOTTING
  %%% Creates or calculates all major and minor path points
  %%% - if we are given a start and an end, make a path from that.
  %%% - if we have no path, create a random one starting from [0,0,0,0].
  %%%%%%%%%%%%
  if jj == 0,
    % If we have start and end goals, create a path between them.
    originposes(:, 1) = poses(:, 1);
    old_poses = poses;
    xs = poses(1, 1);
    ys = poses(2, 1);
    listx = horzcat(listx, xs);
    listy = horzcat(listy, ys);
    for ii = 2:n,
      originposes(:, ii) = poses(:, ii);
      [xs, ys] = FindPath(poses(:,ii-1), poses(:,ii));
      listx = [ listx, xs ];
      listy = [ listy, ys ];
      distances(ii-1, 1) = PathDistance(xs, ys, Map);
    end
    
  else
    x = numel(poses(1, :));
    for ii = 2:x,
      [xs,ys] = FindPath( poses(:, ii-1),  poses(:, ii) );
      listx = [ listx, xs ];
      listy = [ listy, ys ];
      distances(ii-1, 1) = PathDistance(xs, ys, Map);
    end
    set(h, 'Color', 'c'); %Decolorize our previous paths
  end
  
  %gui things
  if gui_on,
    if jj == 0,
      for ii = 1:numel(poses(1, :)),
        h = [h DrawCar( [0.02; 0.05], poses(:, ii), 'b' )];
      end
      h = [h plot( listx, listy, 'b-' )];
      h = [h, DrawContour(Map, false)];
      axis([min(listx) max(listx) min(listy) max(listy)]);
      drawnow;
      waitforbuttonpress;
    end
    if mod(jj,20)==0, %Only draw our paths every 20 iterations
      for ii = 1:numel(poses(1, :)),
        h = [h DrawCar( [0.02; 0.05], poses(:, ii), 'b' )];
      end
      axis([min(listx) max(listx) min(listy) max(listy)]);
      h = [h plot( listx, listy, 'b-' )];
      h = [h, DrawContour(Map, false)];
      drawnow;
    end
  end
  
  %%%%%%%%%%%%
  %%% CALCULATE JACOBIANS
  %%% Perturbs each point by a small amount, to populate our Jacobian matrix
  %%%%%%%%%%%%
  J = GetJacobians( poses, distances, Map );
  [norm_Deltas, VarDeltas] = GaussNewton(J, distances);
  
  
  % Just in case we add more parameters.
  lcol = numel(poses(1, :)); 
  lrow = numel(poses(:, 1))-1;
  real_old_poses = old_poses;
  old_poses = poses;
  for rr = 1:lcol,
    for ss = 1:lrow,
      poses(ss, rr) = poses(ss, rr)+VarDeltas(vars*(rr-1)+ss, 1);
    end
  end
  
  % If we're NOT at a stopping point, then find our current norm and dist and
  % keep going. 
  if norm_Deltas<prev_norm_Deltas,
    prev_norm_Deltas = norm_Deltas;
  end
  if old_dist > sum(distances),
    old_dist = sum(distances);
    hit_min = 0;
  else
    hit_min = hit_min+1;
  end  
  if etime(c2, c1)>1,
    disp('.');
    c1 = clock;
  end
  
  %%%%%%%%%%%%
  %%% CHECK GAUSS-NEWTON | CREATE 2D COMMANDS IF PATH IS MINIMIZED
  %%%%%%%%%%%% 
  if jj == maxIter || norm_Deltas < 1e-3 || hit_min == 1,
    poses = real_old_poses;
    listx = [];
    listy = [];
    disp('Path is optimized!');
    for ii = 2:numel(poses(1, :)),
      [xs,ys] = FindPath( poses(:, ii-1),  poses(:, ii) );
      listx = [ listx, xs ];
      listy = [ listy, ys ];
      x_poses = [x_poses; {xs}];
      y_poses = [y_poses; {ys}];
      th_poses = [th_poses; FindThetas(poses(:, ii-1), xs, ys)];
    end
    for i = 1:numel(th_poses),
      turns = th_poses{i};
      poses(3, i) = turns(1);
      poses(3, i+1) = turns(end);
      major_poses(:, i:i+1) = [poses(1:3, i), poses(1:3, i+1)];
    end
    if gui_on,
      delete(h);
      for ii = 1:numel(poses(1,:)),
        h = DrawCar( [0.02; 0.05], poses(:, ii), 'b' );
      end
      h = [h plot( listx, listy, 'b-' )];
      h = [h, DrawContour(Map, false)];
    end
    return;
  end
 
   
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% HELPER FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xf] = ApplyCommands( xi, t, v, w )
dt = diff(t);
dt = dt(1);
xp = xi(1);
yp = xi(2);
tp = xi(3);
for j = 1:numel(t)
  tp = [tp, tp(end) + w(j)*dt];
  xp = [xp, xp(end) + v(j)*cos(tp(end))*dt];
  yp = [yp, yp(end) + v(j)*sin(tp(end))*dt];
end
xf = [xp; yp; tp];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xs, ys] = FindPath( startpt, endpt )
[ A, Knots ] = Bezier_curves(startpt, endpt);
xs = A(1,:);
ys = A(2,:);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ dist ] = PathDistance(xs, ys, Map)
%%Just the distance of the path. Paths are given in a series of
%%connecting points.
dist = 0;
for i = 1:numel(xs)-1,
  dist = dist + norm( abs([xs(i+1)-xs(i), ys(i+1)-ys(i)]), 2 );
end
dist = dist + (Map.height_fun(xs(end), ys(end)) - Map.height_fun(xs(1), ys(1)));

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [norm_Deltas, Deltas] = GaussNewton(J, Resid)
%%Gauss-Newton matrix multiplication.
damp = .5;
A = (J'*J);
B = J'*Resid;
Deltas = pinv(A)*B;
while abs(max(Deltas))>.1,
  Deltas = Deltas*damp;
end
norm_Deltas = norm(abs(Deltas), 2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ J ] = GetJacobians( poses, distances, Map )
% Goes through all of the elements of poses and wiggles them.
delta = 1e-9;
J = zeros(numel(distances), numel(poses(1:3, :)));
lcol = numel(poses(1, :));
lrow = numel(poses(:, 1))-1;
for rr = 1:lcol,
  for ss = 1:lrow,
    xfjac = poses;
    if (rr == 1 && (ss == 1 || ss == 2)) || (rr == lcol && (ss == 1 || ss == 2))
    else
      xfjac(ss, rr) = poses(ss, rr)+delta;
      
      if rr == 1,
        [xs1,ys1] = FindPath( xfjac(:, rr), xfjac(:, rr+1) );
        newdistance = PathDistance(xs1, ys1, Map);
        J(rr, 3*rr+(ss-3)) = (distances(rr, 1)-newdistance)/delta;
        
      elseif rr == lcol,
        [xs1,ys1] = FindPath( xfjac(:, rr-1), xfjac(:, rr) );
        newdistance = PathDistance(xs1, ys1, Map);
        J(rr-1, 3*rr+(ss-3)) = (distances(rr-1, 1)-newdistance)/delta;
        
      else
        %The route before the point
        [xs,ys] = FindPath( xfjac(:, rr-1), xfjac(:, rr) );
        newdistance1 = PathDistance(xs, ys, Map);
        J(rr-1, 3*rr+(ss-3)) = (distances(rr-1, 1)-newdistance1)/delta;
        
        %The route after the point; change the point after to
        %compensate for the way the splines are computed.
        [xs2,ys2] = FindPath( xfjac(:, rr), xfjac(:, rr+1) );
        newdistance2 = PathDistance(xs2, ys2, Map);
        J(rr, 3*rr+(ss-3)) = (distances(rr, 1)-newdistance2)/delta;
      end
    end
  end
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function map = CreateHeightmap(max_x, min_x, max_y, min_y, max_ht, granularity)
width = max_x-min_x;
length = max_y-min_y;
x = rand(500, 1)*width;
y = rand(500, 1)*length;
%These commands center our plot.
z = rand(500, 1)*max_ht;
xlin = linspace(min(x), max(x), width*granularity);
ylin = linspace(min(y), max(y), length*granularity);
[X,Y]=meshgrid(xlin,ylin);
f = scatteredInterpolant(x, y, z);
Z = f(X,Y);
map.height_fun = f;
map.x_vals = X+min_x;
map.y_vals = Y+min_y;
medz = (max(max(Z)) + min(min(Z)))/2;
map.z_vals = Z-repmat(medz, size(Z));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function h = DrawContour(map, boolDrawLabel)
%A handy method to see the contour map the mesh produced.
[C,h]=contour(map.x_vals,map.y_vals,map.z_vals);
if boolDrawLabel==true,
  clabel(C,h,'FontSize', 10, 'Color', 'r', 'Rotation', 0);
end
end


