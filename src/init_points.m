%
% PURPOSE:
%    Generate points defining the environment features.
%    For the random points generation, we start with uniform distributions.
%    But the polar coordinates that are used warps this into a nonuniform distribution in 2D.
%
% USAGE:
%    all_points = init_points()                % Generates 4 points deterministically
%    all_points = init_points(num_pts, border) % Generates "num_pts" random points within border.
%
% INPUTS:
%    num_pts  - number of random points generated in outer border in unit square.
%    border   - thickness or border just on the inside of the unit square.
%
% OUTPUTS:
%    all_points - 2 x num_pts  array of output points.
%
% by Arjang Hourtash

function all_points = init_points( num_pts, border )

center = 0.5;

% Define environment.
if( ~exist( 'num_pts', 'var' ) || isempty( num_pts ) )
    % Use predefined points, good for comparisons against mutiple runs.
    all_points = [0.1, 0.1
                  0.8, 0.1
                  0.5, 0.7
                  0.2, 0.8]'; % 4 obstacles defined as points.
elseif( (num_pts <= 0) || ( border < 0 ) || ( border > center ) )
    error( 'Inputs must be as follows: num_pts >= 1. border in [0, 0.5].' );
else
    % Generate random points. Good for comparisons against multiple environments.
    lo = center - border;
    hi = center;

    radius = rand(1, num_pts);
    angles = rand(1, num_pts) * (2*pi);

    radius_lo = min( [abs( lo ./ cos( angles ) ); abs( lo ./ sin( angles ) )], [], 1 );
    radius_hi = min( [abs( hi ./ cos( angles ) ); abs( hi ./ sin( angles ) )], [], 1 );
    pts = [cos(angles); sin(angles)] .* ([1;1] * (radius_lo + radius .* (radius_hi - radius_lo) ));

    % Scale radii to land everything in border.
    all_points = center * ones(2,size(pts, 2)) + pts;
end

end

% Map the [-1,1] filled square into a border area.
function pts = scale_pts( pts, lo, hi )


end
