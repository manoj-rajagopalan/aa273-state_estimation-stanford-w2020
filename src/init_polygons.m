% Initialization script for all polygons defining the environment boundary,
% and the obstacles.
function all_polygons = init_polygons()

% Useful constants.
sqrt3o2 = sqrt(3)/2;
d2r     = pi/180;

% Reference objects.
square = [0, 0
          1, 0
          1, 1
          0, 1]';
triangle = [0, 0
            1, 0
            0.5, sqrt3o2]';
pentagon = [0, 0
            1, 0
            1.5, 1
            0.5, 2
           -0.5, 1]';

% Define environment.
test_boundary = polygon(   square*1.0 ,  0*d2r, [0  ; 0  ] );
obstacles(1)  = polygon( triangle*0.1 , 45*d2r, [0.1; 0.1] );
obstacles(2)  = polygon(   square*0.1 , 10*d2r, [0.8; 0.1] );
obstacles(3)  = polygon( pentagon*0.1 ,  0*d2r, [0.5; 0.7] );
obstacles(4)  = polygon( pentagon*0.1 , 60*d2r, [0.2; 0.8] );
all_polygons = [test_boundary, obstacles];

end
