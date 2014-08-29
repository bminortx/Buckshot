function [ forces, steering_angs, Knots ] = RunSplineFunction(Vehicle,...
  startpt, endpt)
% spline_params, 
%RUNSPLINEFUNCTION Summary of this function goes here
%   Detailed explanation goes here

[ A, Knots ] = Bezier_curves(startpt, endpt);
xs = A(1,:);
ys = A(2,:);
ths = FindThetas( startpt, xs, ys );
[ forces, steering_angs ] = CalculateCommands( Vehicle, xs, ys, ths, startpt(4), endpt(4) );
% forces = Cell2Matrix(force)];
% steering_angs = Cell2Matrix(steering)];

end

