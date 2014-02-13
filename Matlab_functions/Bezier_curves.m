function [ Curve, Knots ] = Bezier_curves(xi, xf)
% Bezier_curves takes a series of knots and transforms them into their 
% representative Bezier curve. 
%  1. Bezier_curves(xi, xf): xi and xf are both [x, y, theta, vel] vectors.
%  Curvature can be added as a last component, [x, y, th, vel, k], but it is not
%  necessary.
%
% There is one return statement: 
%  1. Curve = list of [x;y] points that make up the Bezier curve
%
% References: 
%  * Thomas W. Sederberg; Computer Aided Geometric Design
%  * Nima Keivan and Gabe Sibley; Realtime Simulation-in-the-loop Control for 
%     Agile Ground Vehicles, Appendix A. 

%%%%%%%
% Parse our arguments
% %%%%%%%
% if nargin==3, 
%   xi = varargin{1};
%   xf = varargin{2};
%   xf(3) = xf(3)-pi;
%   distance = norm(abs([xf(2)-xi(2), xf(1)-xi(1)]), 2);
%   Params = Cell2Matrix(varargin{3});
%   ai = Params(1);
%   bi = Params(2);
%   af = Params(3);
%   bf = Params(4);
% elseif nargin==2, 

xf(3) = xf(3)-pi;
distance = norm(abs([xf(2)-xi(2), xf(1)-xi(1)]), 2);
ai = distance*(1/5);
bi = ai; %Extension of ai; projection of horz. from P1 to P2
af = ai;
bf = ai;

% end

% Params = [ai, bi, af, bf];

%%%%%
% Constants
%%%%%
n = 6;
delta_t = 1/30;
d = distance; % Wrong; Total length of spline. Best way to do this might be simulation...
total_t = 2*d/(xi(4)+xf(4));
time_seg = delta_t/total_t;

% Calculate k and h
if numel(xi)==5 && numel(xf)==5,
  ki = xi(5);
  kf = xf(5);
else
  % I just whipped up a little sumpin' hurr.
  ki = abs(( xf(3)-xi(3) ) / ( xf(4) * total_t * 3 ));
  kf = abs(( xf(3)-xi(3) ) / ( xf(4) * total_t * 3 ));
end

hi = (ki*ai^2*n)/(n-1);
hf = (kf*af^2*n)/(n-1);

%%%%%
% Find our knots
%%%%%
Knots = zeros(2, n);
% These depend on the start point.
Knots(:,1) = [xi(1); xi(2)];
Knots(:,2) = [xi(1)+(ai)*cos(xi(3));    xi(2)+(ai)*sin(xi(3))];
% These depend on the end. 
Knots(:,5) = [xf(1)+(af)*cos(xf(3));    xf(2)+(af)*sin(xf(3))];
Knots(:,6) = [xf(1); xf(2)];
% These must be determined from two options. 
Pt_31 = RotTransPoint([(ai+bi); hi], xi(3), xi(1), xi(2));
Pt_32 = RotTransPoint([(ai+bi); -hi], xi(3), xi(1), xi(2));
Pt_41 = RotTransPoint([(af+bf); hf], xf(3), xf(1), xf(2));
Pt_42 = RotTransPoint([(af+bf); -hf], xf(3), xf(1), xf(2));
if norm(abs(Knots(:,6)-Pt_31)) < norm(abs(Knots(:,6)-Pt_32)),
  Knots(:,3) = Pt_31;
else
  Knots(:,3) = Pt_32;
end
if norm(abs(Knots(:,1)-Pt_41)) < norm(abs(Knots(:,1)-Pt_42)),
  Knots(:,4) = Pt_41;
else
  Knots(:,4) = Pt_42;
end

%%%%%%
% Plot our curves
%%%%%%
s = 0:time_seg:1;
Curve = zeros(2, numel(s));

% Make a table for nchoosek, since that apparently takes forever.
choose = zeros(1, n+1);
for ii = 1:n+1, 
  choose(ii) = nchoosek(n, ii-1);
end

for u = 1:numel(s),
  sum = [0; 0];
  for ii = 1:n,
    B = choose(ii) * s(u)^(ii-1) * (1-s(u))^(n-ii+1);
    sum = sum + B*Knots(:,ii);
  end
  B = choose(n+1)*(s(u)^n);
  sum = sum + B*Knots(:,n);
  Curve(:, u) = sum;
end


end

function [new_pt] = RotTransPoint(pt, theta_rad, x_trans, y_trans)
new_pt = [cos(theta_rad) -sin(theta_rad); sin(theta_rad) cos(theta_rad)]*[pt(1); pt(2)];
new_pt(1) = new_pt(1)+x_trans;
new_pt(2) = new_pt(2)+y_trans;
end

