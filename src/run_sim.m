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
noise_process = mvnrnd( zeros(1, num_states), Qmat, sim_num_iter )';

% Measurement noise.
Rmat = diag( reshape( [0.25; 0.1]*ones(1, num_obst), [], 1 ) ); % Variances for bearing, range.
noise_measure = [zeros(num_measure, 1), mvnrnd( zeros(1, num_measure), Rmat, sim_num_iter )']; % 1st column at t=0.

if( true_for_live_plot )
    % Initialize plots.
    [f1, f2] = init_plot_scene();
end

% Initialize simulation and filtering.
% Memory allocation for speed.
% In the following iteration 1 represents t=0.
Xt_actual = NaN(num_states , sim_num_iter+1);
Xt_ekf    = NaN(num_states , sim_num_iter+1);
Xt_ukf    = NaN(num_states , sim_num_iter+1);
Xt_fast   = NaN(num_states , sim_num_iter+1);
Yt        = NaN(num_measure, sim_num_iter+1);

% Computation time sums.
flop_time_ekf  = 0;
flop_time_ukf  = 0;
flop_time_fast = 0;

% State Initialization.
X_obst = reshape( all_points, [], 1);
Xt_actual(:,1) = [rbt.get_state(); X_obst]; % Xt_actual at t=0.

% EKF: Initialization.
% TBD
% TBD
% TBD

% UKF: Initialization.
Xt_ukf(:,1) = Xt_actual(:,1) + mvnrnd( zeros(1, num_states), Qmat, 1 )'; % t=0.
sig_ukf = 100*eye(num_states); % Large uncertainty.
args_FofX  = {rbt};
args_GofX  = {pfinder};
ukf_lambda = 2;

% FastSLAM: Initialization.
% TBD
% TBD
% TBD

% Simulation iterations.
for idt = 2:(sim_num_iter+1)

    % PROCESS: Robot dynamics: X_t = f( X_t-1, u_t-1 )
    Xt_actual(:,idt) = slam_FofX( Xt_actual(:,idt-1), Ut, rbt ) ...
                     +        noise_process(:,idt-1);

    % MEASUREMENT: Range finder: Y_t = g( X_t ).
    Yt(:,idt) = slam_GofX( Xt_actual(:,idt), Ut, pfinder ) ...
              + noise_measure(:,idt);

    if( true_for_live_plot )
        % Plots.
        plot_scene_points( f1, f2, all_points, rbt, pfinder, v_bearing_rad_rel, v_range );
        fprintf( 'Press <enter> to advance to the next frame.\n' );
        pause;
    end

    % --------------
    % FILTERS:
    % --------------

    % EKF:
    tic;
    % Insert filter function here.
    % TBD
    % TBD
    % TBD
    flop_time_ekf = flop_time_ekf + toc;


    % UKF:
    tic;
    [sig_ukf, Xt_ukf(:,idt)] = unscented_kalman_filter( sig_ukf, Xt_ukf(:,idt-1), ...
                                   @slam_FofX, args_FofX, ...
                                   @slam_GofX, args_GofX, ...
                                   Qmat, Rmat, Ut, Yt(:,idt), ukf_lambda );
    flop_time_ukf = flop_time_ukf + toc;


    % FastSLAM:
    tic;
    % Insert filter function here.
    % TBD
    % TBD
    % TBD
    flop_time_fast = flop_time_fast + toc;

end

fprintf( 'Computations times (cpu seconds per time step) are as follows:\n' );
fprintf( 'EKF : %f\n', flop_time_ekf  / sim_num_iter );
fprintf( 'UKF : %f\n', flop_time_ukf  / sim_num_iter );
fprintf( 'Fast: %f\n', flop_time_fast / sim_num_iter );

% Final result plots.
plot_results;

end
