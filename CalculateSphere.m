function CalculateSphere[radius, height]




[oldx,oldy,oldz] = sphere;
% x = [];
% y = [];
% z = [];
x = this.radius*oldx;
y = this.radius*oldy;
z = this.radius*oldz;
%       for i=1:numel(oldx(:,1)),
%         if mod(i, 2)==1,
%           x = [x; oldx(i, :)];
%           y = [y; oldy(i, :)];
%           z = [z; oldz(i, :)];
%         end
%       end


surf(x,y,z, 'FaceColor', 'r');
%   hsurf = surf(x,y,z, 'FaceColor', 'r');

end