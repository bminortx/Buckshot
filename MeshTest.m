width = 2; 
length = 2;
height = 1;
granularity = 1;

Y = reshape(Y, numel(Y), 1);


x y = meshgrid 0:1/scale 



x = rand(100, 1)*width;
y = rand(100, 1)*length;
z = rand(100, 1)*height;
xlin = linspace(min(x), max(x), width*granularity);
ylin = linspace(min(y), max(y), length*granularity);
scale = 20;
[X,Y]=meshgrid([0:1/scale:2],[0:1/scale:2]);
f = scatteredInterpolant(x, y, z);
Z = f(X,Y);
surf(X, Y, Z)x = 

reshape(x, numel(x), 1);
reshape(y, numel(y), 1);
z = randn(size(x));
surf(x, y, z)

scale = 2;