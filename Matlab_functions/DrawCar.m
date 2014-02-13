
% DrawCar( m, q )
%
% m = [ Wheelbase; Width ]
% q = [ X;Y; Heading; SteeringAngle ]
function h = DrawCar( m, q, color )

    if nargin ~= 3
        color = 'b';
    end

    % x forward
    % y right
    L = m(1);
    W = m(2);
    P = q(1:2);

    R = [cos(q(3)), -sin(q(3)); sin(q(3)), cos(q(3))];
    
    Wheels = R*L*[ 1, 1, -1, -1;...
                  -1, 1,  1, -1];

    Wheels = [Wheels(1,:) + P(1); Wheels(2,:) + P(2)];
    
    h = plot( Wheels(1,3:4), Wheels(2,3:4), '-', 'Color', color );
    h = [h plot( Wheels(1,1:2), Wheels(2,1:2), '-', 'Color', color )];

    fcenter = (Wheels(:,1) + Wheels(:,2))/2;
    rcenter = (Wheels(:,3) + Wheels(:,4))/2;

    h = [ h plot( [fcenter(1),rcenter(1)], [fcenter(2),rcenter(2)], '-', 'Color', color ) ];
    h = [ h plot( rcenter(1), rcenter(2), 'o', 'Color', color ) ];
	h = [ h PlotWheel( [Wheels(:,1); q(3)+q(4)], L, color ) ];
	h = [ h PlotWheel( [Wheels(:,2); q(3)+q(4)], L, color ) ];
	h = [ h PlotWheel( [Wheels(:,3); q(3)], L, color ) ];
	h = [ h PlotWheel( [Wheels(:,4); q(3)], L, color ) ];

function h = PlotWheel( q, s, color )
    % front left, front right, back right, back left
    wheel = s*[  0.5, 0.5, -0.5, -0.5, ;...
               -0.25, 0.25,  0.25, -0.25, ];
    R = [cos(q(3)), -sin(q(3)); sin(q(3)), cos(q(3))];
    wheel = R*wheel;
    wheel = [wheel(1,:)+q(1);wheel(2,:)+q(2)];
    h = plot( [wheel(1,:),wheel(1,1)], [wheel(2,:),wheel(2,1)], '-', 'Color', color );
