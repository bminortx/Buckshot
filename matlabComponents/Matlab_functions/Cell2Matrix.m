function [ matrix ] = Cell2Matrix( cell )
%CELL2MATRIX Summary of this function goes here
%   Detailed explanation goes here
matrix = [];

if iscell(cell),
  for i=1:numel(cell),
    sub_cell= cell{i};
    matrix = [matrix sub_cell];
  end
else
  matrix = cell;
end

end

