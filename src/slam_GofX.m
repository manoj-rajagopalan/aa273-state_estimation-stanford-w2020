%
% PURPOSE:
%    Measurement for simulataneous localization and mapping.
%    Interwomen [relative bearings (radians), range] one pair at a time.
%
%    Note that "U_t" is not strictly needed.  But it is here as an input
%    so that it matches the first 2 inputs of "slam_FofX()" and allows
%    more general filter functions.
%
% USAGE:
%    Y_t = slam_GofX( X_t, U_t, pfinder, all_constants )
%
% INPUTS:
%    X_t           - Nx1 state vector at time t.
%    U_t           - Mx1 input vector at time t.
%    pfinder       - instance of "point_finder.m" class.
%    all_constants - 2xM [XY] coordinates of features in the environment
%                        considered to be known or constant.
%
% OUTPUTS:
%    Y_t       - Mx1 measurement vector at time t.
%
% by Arjang Hourtash

function Y_t = slam_GofX( X_t, U_t, pfinder, all_constants )

% Unpack X_t.
robot_position = X_t(1:2);
robot_orientation = X_t(3);
all_points = reshape( X_t(4:end), 2, [] );
all_points = [all_points, all_constants];

% Measure.
[v_bearing_rad_rel, v_range] = pfinder.range_map( all_points, robot_position, robot_orientation );
Y_t = reshape( [v_bearing_rad_rel; v_range], [], 1);

end
