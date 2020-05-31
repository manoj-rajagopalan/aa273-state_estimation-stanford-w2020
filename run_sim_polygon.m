%
% Usage:
%    run_sim_polygon()   % Nominal use.
%    run_sim_polygon(1)  % Debug mode for range finder.
%
% Note:
%    This simulation has been provided without estimation filters.
%    For your development, please insert your estimation filter
%    where the comments near the bottom indicate.
%
function run_sim_polygon( debug )

if( ~exist( 'debug', 'var' ) || isempty( debug ) )
    debug = 0;
end

% Useful constants.
sim_t_final  = 1.0;
sim_dt       = 0.01;
sim_num_iter = round( sim_t_final / sim_dt );

% Initialize the polygons in the environment.
all_polygons = init_polygons();

% Initialize Robot and Range Finder.
rbt = robot( sim_dt );
turn_radius = 0.2;
rbt = rbt.set_state( [0.5 + turn_radius; 0.4; pi/2] );
Ut = [2*pi*turn_radius; 2*pi]; % Constant input.
rfinder = range_finder( 200 ); % Number of divisions in 360 degrees.

% Initialize plots.
[f1, f2] = init_plot_scene();

% Simulation iterations.
for idt = 1:sim_num_iter

    % PROCESS: Robot dynamics: X_t = f( X_t-1, u_t-1 )
    rbt = rbt.state_transition( Ut );

    % MEASUREMENT: Range finder: Y_t = g( X_t )
    [robot_orientation, robot_position] = rbt.get_pose();
    m_range = rfinder.range_map( all_polygons, robot_position, robot_orientation );

    % Plots.
    plot_scene_polygons( f1, f2, all_polygons, rbt, rfinder, m_range, debug );
    fprintf( 'Press <enter> to advance to the next frame.\n' );
    pause;

    % FILTERS:
    % TBD: Insert filters here.
    % TBD: Insert filters here.
    % TBD: Insert filters here.

end

end
