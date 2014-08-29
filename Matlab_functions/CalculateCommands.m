function [ engine_force, steering_angle ] = CalculateCommands( Vehicle, x_f, ...
  y_f, th_f, start_vel, end_vel )
% CALCULATECOMMANDS interpolates the commands our car needs to drive, based off
% of the x and y positions given. 
% Reference: van der Berg et al, 'Motion planning under uncertainty using 
% iterative local optimization in belief space'

%%% Constants and Data Sets
d = Vehicle.body.length;
vehicle_mass = Vehicle.body.mass;
tau = 1/30; %This is constant throughout the program... 
engine_force = {};
steering_angle = {};

if iscell(x_f), 
  %%% Going through the cells:
  for j = 1:numel(x_f),
    sub_x_f = x_f{j};
    sub_y_f = y_f{j};
    sub_th_f = th_f{j};
    sub_steering_angle = zeros(1, numel(sub_x_f)-1);
    
    % Engine force should be one number.
    force = (vehicle_mass*((end_vel-start_vel)/(tau*numel(sub_x_f)))-GravityCompensation(Vehicle))/2;
%     force = CorrectForce(Vehicle, force, end_vel);
    sub_engine_force = repmat(force, 1, numel(sub_x_f)-1);
    
    % Steering angle should be derivable from the splines
    for i = 2:numel(sub_th_f),
      sub_steering_angle(i-1) = atan2( d * ((sub_th_f(i) - sub_th_f(i-1) )), d * end_vel * tau);
    end
    sub_steering_angle = CorrectSteering(Vehicle, sub_steering_angle);
    engine_force = [engine_force; {sub_engine_force}];
    steering_angle = [steering_angle; {sub_steering_angle}];

  end
else
    force = (vehicle_mass*((end_vel-start_vel)/(tau*numel(x_f)))-GravityCompensation(Vehicle))/2;
%     force = CorrectForce(Vehicle, force, end_vel);
    engine_force = repmat(force, 1, numel(x_f)-1);
    steering_angle = zeros(1, numel(x_f)-1);
    for i = 2:numel(th_f),
      steering_angle(i-1) = atan2( d * ((th_f(i) - th_f(i-1) )), d * end_vel * tau);
    end
%     steering_angle = CorrectSteering(Vehicle, steering_angle);
end

end

function Gforce = GravityCompensation(Vehicle)
rotation = Vehicle.GetRotation();
Gforce = dot([0 0 -10], rotation(:,1))/Vehicle.body.mass;
end

function Fforce = FrictionCompensation(Vehicle)


end

function correct_force = CorrectForce(Vehicle, force, end_vel)

% torque = Pwm*Ts - slope*V
force = force - Vehicle.params.accel_offset
stallTorque = force * Vehicle.params.stall_torque_coeff
correction = abs(stallTorque) - Vehicle.params.torque_speed_slope * abs(norm(end_vel))
correct_force = sign(stallTorque) * max([0, correction])
correct_force = correct_force/2

end

function steering_ang = CorrectSteering(Vehicle, steering)

steering = steering - Vehicle.params.steering_offset;
steering = SoftMinimum(Vehicle.params.max_steering, SoftMaximum(steering, -Vehicle.params.max_steering, 10), 10);
steering_ang = steering;
% Set Ackerman Steering later
% double dCurrentSteering = pWorld->m_pVehicle->GetAckermanSteering();
% double dRate = (dCorrectedPhi-dCurrentSteering)/dT;
% //clamp the rate
% dRate = sgn(dRate) * std::min(fabs(dRate),pWorld->m_Parameters[CarParameters::MaxSteeringRate]);
% //apply the steering
% dCorrectedPhi = dCurrentSteering+dRate*dT;

end

function [maxi] = SoftMaximum(x, y, multiplier)
x = x*multiplier;
y = y*multiplier;
maximum = max(x, y);
minimum = min(x, y);
maxi = (log1p(exp(minimum-maximum)) + maximum)/multiplier;
end

function [min] = SoftMinimum(x, y, multiplier)
min = -SoftMaximum(-x, -y, multiplier);
end



