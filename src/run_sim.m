%
% USAGE:
%    run_sim( true_for_live_plot )
%
% INPUTS:
%    true_for_live_plot : 0 = run in batch mode.
%                         1 = step through each time step.
%
% DEFINITIONS:
%    Xt : State vector = [  3x1 robot [Px; Py; theta]              ]
%                        [ 2Nx1 obstacle [Px; Py] for N obstacles. ]
%
%    Yt : [ 2Mx1 obstacle [relative bearing (radians); range] ]
%         [      repeated and interleaved for M obstacles     ]
%
%    The following are represented as Nx(T+1),
%        where T is max simulation time steps
%        First column represents states at t=0.
%
%    Xt_actual                - simulation states
%    Xt_ekf, Xt_ukf, Xt_fast  - filter states
%
%    Qmat : NxN process     noise covariance matrix.
%    Rmat : MxM measurement noise covariance matrix.
%
% NOTE:
%    This simulation has been provided without estimation filters.
%    For your development, please insert your estimation filter
%    where the comments near the bottom indicate.
%
function run_sim( true_for_live_plot )

% Useful constants.
sim_t_final  = 1.0;
sim_dt       = 0.01;
sim_num_iter = round( sim_t_final / sim_dt );
tvec = linspace( 0, sim_t_final, sim_num_iter+1 )';

% Initialize the points in the environment.
all_points = init_points();
num_obst = size( all_points, 2 );
num_states_obst = numel( all_points );

% Initialize Robot.
rbt = robot( sim_dt );
num_states_rbt = length( rbt.get_state() );
turn_radius = 0.2;
rbt = rbt.set_state( [0.5 + turn_radius; 0.4; pi/2] ); % Initial state.
Ut = [2*pi*turn_radius; 2*pi]; % Constant input.

% Initialize point finder.
pfinder = point_finder();
num_measure = 2*num_obst;

% Process noise. Noisy robot dynamics. But obstacle positions don't evolve, i.e. without noise.
num_states = num_states_rbt + num_states_obst;
Qmat = zeros(num_states, num_states);
% Robot specific process noise:
Qmat(1:num_states_rbt, 1:num_states_rbt) = sim_dt * diag([0.1, 0.1, 0.2]); % For [Px, Py, theta]
% Generate process noise for simulation.
noise_process = mvnrnd( zeros(1, num_states), Qmat, sim_num_iter+1 )'; % +1 so array sizes match.

% Measurement noise.
Rmat = diag( reshape( [0.25; 0.1]*ones(1, num_obst), [], 1 ) ); % Variances for bearing, range.
noise_measure = mvnrnd( zeros(1, num_measure), Rmat, sim_num_iter+1 )'; % +1 so array sizes match.

if( true_for_live_plot )
    % Initialize plots.
    [f1, f2] = init_plot_scene();
end

% Initialize simulation and filtering.
% Memory allocation for speed.
% In the following iteration 1 represents t=0.
Xt_actual = NaN(num_states, sim_num_iter+1);
Xt_ekf    = NaN(num_states, sim_num_iter+1);
Xt_ukf    = NaN(num_states, sim_num_iter+1);
Xt_fast   = NaN(num_states, sim_num_iter+1);
Yt        = NaN(num_measure, sim_num_iter+1);

% Initialization.
X_obst = reshape( all_points, [], 1);
Xt_actual(:,1) = [rbt.get_state(); X_obst]; % Xt_actual at t=0.

% Simulation iterations.
for idt = 2:(sim_num_iter+1)

    % PROCESS: Robot dynamics: X_t = f( X_t-1, u_t-1 )
    rbt = rbt.state_transition( Ut );
    Xt_actual(:,idt) = [rbt.get_state(); X_obst] + noise_process(:,idt);

    % MEASUREMENT: Range finder: Y_t = g( X_t ). Interweave [relative bearing, range] pairs one at a time.
    [robot_orientation, robot_position] = rbt.get_pose();
    [v_bearing_rad_rel, v_range] = pfinder.range_map( all_points, robot_position, robot_orientation );
    Yt(:,idt) = reshape( [v_bearing_rad_rel; v_range], [], 1) + noise_measure(:,idt);

    if( true_for_live_plot )
        % Plots.
        plot_scene_points( f1, f2, all_points, rbt, pfinder, v_bearing_rad_rel, v_range );
        fprintf( 'Press <enter> to advance to the next frame.\n' );
        pause;
    end

    % FILTERS:
    % TBD: Insert filters here.
    % TBD: Insert filters here.
    % TBD: Insert filters here.

end

% Final result plots.
plot_results;

end
