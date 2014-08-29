function [ lin_vel, ang_vel ] = CalculateVelocities( x_f, y_f, th_f )
%CALCULATEVELOCITIES grabs the linear and angular velocities from our
%SplineOptimizer, to later pass them to our BulletSim.

%%% Constants and Data Sets
tau = 1/30; %This is constant throughout the program... 
lin_vel = {};
ang_vel = {};
last_lin_vel = 0;
last_ang_vel = 0;

%%% Going through the cells:
for j = 1:numel(x_f),
  sub_x_f = x_f{j};
  sub_y_f = y_f{j};
  sub_th_f = th_f{j};
  sub_lin_vel = zeros(1, numel(sub_x_f));
  sub_ang_vel = zeros(1, numel(sub_x_f));
  
  if j == 1, 
    sub_lin_vel(1) = 0;
    sub_ang_vel(1) = 0;
  else
    sub_lin_vel(1) = last_lin_vel;
    sub_ang_vel(1) = last_ang_vel;
  end
  
  % Fill all the state vectors
  for i = 2:numel(sub_x_f),
    sub_ang_vel(i) = ( sub_th_f(i) - sub_th_f(i-1) ) / tau;
    distance = sqrt(( sub_x_f(i)-sub_x_f(i-1) )^2+( sub_y_f(i)-sub_y_f(i-1) )^2);
    sub_lin_vel(i) = ( distance ) / tau;
  end
  last_lin_vel = sub_lin_vel(end);
  last_ang_vel = sub_ang_vel(end);
  
  ang_vel = [ang_vel; {sub_ang_vel}];
  lin_vel = [lin_vel; {sub_lin_vel}];
  
end

end

