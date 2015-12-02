function [ yaw ] = Rot2Yaw( rot_mat )
%ROT2YAW Summary of this function goes here
%   Detailed explanation goes here

yaw = atan2d(rot_mat(3, 1), rot_mat(3, 2));

end

