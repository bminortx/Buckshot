function [ths] = FindThetas( startpt, xs, ys )
ths = zeros(1, numel(xs));
for i=2:numel(xs),
  % For some reason, we have to compensate for some things. 
  ths(i) = atan2( (ys(i)-ys(i-1)), (xs(i)-xs(i-1)) );
end
% Base the first theta on the second one
ths(1) = ths(2);

end