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


%true_for_live_plot = 0;


% Useful constants.
sim_t_final  = 1.0;
sim_dt       = 0.01;
sim_num_iter = round( sim_t_final / sim_dt );
tvec = linspace( 0, sim_t_final, sim_num_iter+1 )';

% Initialize the points in the environment.
all_points = init_points();
num_obst = size( all_points, 2 );
num_states_obst = numel( all_points );

% %
% % Leo, 2020/06/04
% %
% % Conver all_points to map structure data type
% map = convertPoints2Map(all_points);
% 
% % Get the number of landmarks
% NLandmarks = size(map,2);



% Initialize Robot.
rbt = robot( sim_dt );
num_states_rbt = length( rbt.get_state() );
turn_radius = 0.2;
rbt = rbt.set_state( [0.5 + turn_radius; 0.4; pi/2] ); % Initial state. = X0
%X0  = rbt.state;      % JDLee, 2020/6/5
Ut = [2*pi*turn_radius; 2*pi]; % Constant input. (constant input?)

% Initialize point finder.
pfinder = point_finder();
num_measure = 2*num_obst;

% Process noise. Noisy robot dynamics. But obstacle positions don't evolve, i.e. without noise.
num_states = num_states_rbt + num_states_obst;
Qmat = zeros(num_states, num_states);
% Robot specific process noise:
Qmat(1:num_states_rbt, 1:num_states_rbt) = sim_dt * diag([0.01, 0.01, 0.02]); % For [Px, Py, theta]
% Generate process noise for simulation.
noise_process = mvnrnd( zeros(1, num_states), Qmat, sim_num_iter )';

% Measurement noise.
Rmat = diag( reshape( [0.002; 0.001]*ones(1, num_obst), [], 1 ) ); % Variances for bearing, range.
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



%
% FastSLAM: Initialization - JDLee, 2020/6/4
%

global numParticles;         % number of particles
global Q;                    % process noise
global R;                    % measurement noise for range and bearing
global dT;                   % simulation time step
global Beta;                 % resample threadhold for ESS [0 1]

numParticles = 35;
dT = sim_dt;
Beta = 0.8;

% Conver all_points to map structure data type
map = convertPoints2Map(all_points);

% Get the number of landmarks
NLandmarks = size(map,2);

% Process noise covariance for particle filters: Q[3X3]
Q = Qmat(1:3,1:3);
R = Rmat(1:2,1:2);
R = zeros(2);
R(1,1) = Rmat(2,2);    % because my measurement model is [range; bearing]
R(2,2) = Rmat(1,1);

% Initialize the Xt_fast
Xt_fast(1:3,1) = rbt.get_state();
Xt_fast(4:11,1) = zeros(8,1);

% Initialize the particles array
for i = 1:numParticles
    particles(i).weight = 1/numParticles;
    %particles(i).pose = zeros(3,1);                % robotX, robotY, theta
    particles(i).pose = rbt.get_state();            % X0 
    particles(i).history = {};                      % empty cell array
    for l = 1:NLandmarks
        particles(i).landmarks(l).observed = false;
        particles(i).landmarks(l).mu = zeros(2,1);  % mjx, mjy
        particles(i).landmarks(l).sigma = zeros(2,2);
    end
end



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
        v_bearing_rad_rel = Yt(1:2:end,idt);
        v_range           = Yt(2:2:end,idt);
        rbt = rbt.set_state( Xt_actual(1:3,idt) );
        plot_scene_points( f1, f2, all_points, rbt, pfinder, v_bearing_rad_rel, v_range );
        fprintf( 'Press <enter> to advance to the next frame.\n' );
        pause;
    end

    % --------------
    % FILTERS:
    % --------------

%     % EKF:
%     tic;
%     % Insert filter function here.
%     % TBD
%     % TBD
%     % TBD
%     flop_time_ekf = flop_time_ekf + toc;


%     % UKF:
%     tic;
%     [sig_ukf, Xt_ukf(:,idt)] = unscented_kalman_filter( sig_ukf, Xt_ukf(:,idt-1), ...
%                                    @slam_FofX, args_FofX, ...
%                                    @slam_GofX, args_GofX, ...
%                                    Qmat, Rmat, Ut, Yt(:,idt), ukf_lambda );
%     flop_time_ukf = flop_time_ukf + toc;
    
    

    % *********************************************************************
    %
    % FastSLAM: J.D. Lee, 2020/6/5
    %
    % *********************************************************************
    
    tic;
    
    % Prediction step of particle filter
    controlInp = Ut;
    particles  = prediction_step(particles, controlInp);
    
    % Perform the correction step of the particle filter
    % Current measurement
    Ytt = Yt(:,idt);
    yt = convertYt(Ytt);
    particles = correction_step(particles, yt);
    
    % TBD
    flop_time_fast = flop_time_fast + toc;
    
    % 
    % Generate visualization plots of the current state of the filter
    %
    pauseTime = 0.1;
    plot_state(particles, map, pauseTime);
    
    Xt_fast(:,idt) = getBestState(particles, NLandmarks);      % Save the best robot state

    % Resample the particle set
    particles    = resample(particles);
    
    % *********************************************************************
    %
    % End of FastSLAM
    %
    % *********************************************************************

end

% Plot the final state of FastSLAM
plot_FinalState(Xt_actual, particles, map);




fprintf( 'Computations times (cpu seconds per time step) are as follows:\n' );
fprintf( 'EKF : %f\n', flop_time_ekf  / sim_num_iter );
fprintf( 'UKF : %f\n', flop_time_ukf  / sim_num_iter );
fprintf( 'Fast: %f\n', flop_time_fast / sim_num_iter );

% Final result plots.
%plot_results;
plotResultsLeo;

end
















