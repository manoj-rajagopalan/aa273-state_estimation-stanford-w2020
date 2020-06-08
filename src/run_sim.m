%
% USAGE:
%    error_scores = run_sim( flag_live_plot, true_for_fully_observable, true_for_dyn_model_error, true_for_uniform_noise_measure )
%
% INPUTS:
%    flag_live_plot : 0 = run in batch mode.
%                     1 = go rhrough each time step, display env only.
%                     2 = go through each time step, display env and point finder.
%                     3 = go through each time step of FastSLAM.
%
%    true_for_fully_observable : false = all obstacles are unknown, part of SLAM.
%                                true  = last 2 obstacles are known,
%                                        all else are part of SLAM.
%                                        The 4 bearing and range measurements from the
%                                        2 known obstacles will allow us to resolve robot pose.
%
%    true_for_dyn_model_error : simulation and filter state transition models:
%                               false = do match
%                               true  = don't match
%
%    true_for_uniform_noise_measure : change distribution for measurement noise.
%                                     false = filter:Gaussian, simulation:uniform.
%                                     true  = filter:Gaussian, simulation:Gaussian.
%
% OUTPUTS:
%    error_scores - structure of error scores for the various filters and algorithms.
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
function error_scores = run_sim( ...
                  flag_live_plot, ...
                  true_for_fully_observable, ...
                  true_for_dyn_model_error, ...
                  true_for_uniform_noise_measure )

% Defaults.
if( ~exist( 'flag_live_plot', 'var' ) || isempty( flag_live_plot ) )
    flag_live_plot = 0;
end
if( ~exist( 'true_for_fully_observable', 'var' ) || isempty( true_for_fully_observable ) )
    true_for_fully_observable = false;
end
if( ~exist( 'true_for_dyn_model_error', 'var' ) || isempty( true_for_dyn_model_error ) )
    true_for_dyn_model_error = false;
end
if( ~exist( 'true_for_uniform_noise_measure', 'var' ) || isempty( true_for_uniform_noise_measure ) )
    true_for_uniform_noise_measure = false;
end

% Useful constants.
sim_t_final  = 1.0;
sim_dt       = 0.01;
sim_num_iter = round( sim_t_final / sim_dt );

% Initialize the points in the environment.
flag_4_deterministic_features = 1;
if( flag_4_deterministic_features )
    all_points = init_points();
else
    all_points = init_points(10, 0.2);
end
all_constants = [];
if( true_for_fully_observable )
    all_constants = all_points(:,(end-1):end);
    all_points = all_points(:,1:(end-2));
end
num_obst = size( [all_points, all_constants], 2 );
num_states_obst = numel( all_points );

% Initialize Robot.
rbt = robot( sim_dt );
num_states_rbt = length( rbt.get_state() );
turn_radius = 0.2;
rbt = rbt.set_state( [0.5 + turn_radius; 0.4; pi/2] ); % Initial state.
Ut = [2*pi*turn_radius; 2*pi]; % Constant input.
vel_factor = [1;1];
if( true_for_dyn_model_error )
    % Skew the model the simulation uses relative to the filters.
    % Effectively, add a slight rotational velocity scale, as if one of
    % of the robot's tires is a bit flat.
    vel_factor = [1;2];
end

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
% In the following simulation noise trace, 1st column at t=0.
if( true_for_uniform_noise_measure )
    % Generate a uniform noise trace for the simulation.
    % Scale the uniform trace so that 95% of the uniform distributionn fits the 95% (i.e. 2 sigma) Gaussian size.
    gaussian_to_uniform = reshape( sqrt( diag( Rmat ) ), [], 1 ) * 2 / 0.95;
    uni_scale = (gaussian_to_uniform * ones(1,sim_num_iter));
    uni_normalized = (-1 + 2*rand( [num_measure, sim_num_iter] ));
    uniform_trace = uni_scale .* uni_normalized;
    noise_measure = [zeros(num_measure, 1), uniform_trace];
else
    % Generate a Gaussian noise trace for the simulation.
    noise_measure = [zeros(num_measure, 1), mvnrnd( zeros(1, num_measure), Rmat, sim_num_iter )'];
end

% Initialize plots.
[f1, f2] = init_plot_scene(flag_live_plot);

% Initialize simulation and filtering.
% Memory allocation for speed.
% In the following iteration 1 represents t=0.
Xt_actual = NaN(num_states , sim_num_iter+1);
Xt_ekf    = NaN(num_states , sim_num_iter+1);
Xt_ukf    = NaN(num_states , sim_num_iter+1);
Xt_fast   = NaN(num_states , sim_num_iter+1);
Yt        = NaN(num_measure, sim_num_iter+1);

% State Initialization.
X0  = rbt.state;      % JDLee, 2020/6/5
X_obst = reshape( all_points, [], 1);
Xt_actual(:,1) = [rbt.get_state(); X_obst]; % Xt_actual at t=0.

% Miscellaneous initialization.
Xo_filt = Xt_actual(:,1) + mvnrnd( zeros(1, num_states), Qmat, 1 )'; % t=0.
args_FofX  = {rbt};
args_GofX  = {pfinder, all_constants};

% Computation time sums.
flop_time_ekf  = 0;
flop_time_ukf  = 0;
flop_time_fast = 0;
flop_time_pf   = 0;

% EKF: Initialization.
Xt_ekf(:,1) = Xo_filt;
sig_ekf = NaN( num_states, num_states, sim_num_iter+1 );
sig_ekf(:,:,1) = diag( [1*ones(1,num_states_rbt), ...
                     0.01*ones(1,num_states_obst)] ); % t=0.
mu_predict_ekf = NaN( num_states, sim_num_iter+1 );
sig_predict_ekf = NaN( num_states, num_states, sim_num_iter+1 );
Ks_ekf = NaN(1, sim_num_iter+1);
Kp_ekf = NaN(1, sim_num_iter+1);
Ss_ekf = NaN(1, sim_num_iter+1);
Sp_ekf = NaN(1, sim_num_iter+1);

% UKF: Initialization.
Xt_ukf(:,1) = Xo_filt;
sig_ukf = NaN( num_states, num_states, sim_num_iter+1 );
sig_ukf(:,:,1) = diag( [1*ones(1,num_states_rbt), ...
                     0.01*ones(1,num_states_obst)] ); % t=0.
mu_predict_ukf = NaN( num_states, sim_num_iter+1 );
sig_predict_ukf = NaN( num_states, num_states, sim_num_iter+1 );
Ks_ukf = NaN(1, sim_num_iter+1);
Kp_ukf = NaN(1, sim_num_iter+1);
Ss_ukf = NaN(1, sim_num_iter+1);
Sp_ukf = NaN(1, sim_num_iter+1);
ukf_lambda = 2;

% Particle filter initialization
enable_pf = 0;

num_particles = 1000;

Xt_particles = NaN(num_states, num_particles); % update step
Wt_particles = NaN(num_particles);

Xt_pf = NaN(num_states, sim_num_iter+1); % mean of particles in each iteration

pf_rbt_loc_convergence = NaN(sim_num_iter+1, 1); % norm of deviation
pf_rbt_theta_convergence = NaN(sim_num_iter+1, 1); % abs diff of deviations
pf_map_convergence = NaN(sim_num_iter+1, 1); % norm of deviations
pf_det_sigma = NaN(sim_num_iter + 1); % determinant of covariance

Xt_particles = rand(size(Xt_particles));
Xt_particles(3,:) = 2*pi * (Xt_particles(3,:) - 0.5); % adjust theta range
Wt_particles = 1.0/num_particles * ones(num_particles,1);
Xt_pf(:,1) = Xt_particles * Wt_particles; % mean position
Xt_pf(:,1) = mean(Xt_particles,2);

pf_convergence = NaN(num_measure+2, sim_num_iter+1);


% *********************************************************************
%
% FastSLAM Initialization: J.D. Lee, 2020/6/5
%
% *********************************************************************

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
Xt_fast(:,1) = Xo_filt(:,1);

% Initialize the particles array
for i = 1:numParticles
    particles(i).weight = 1/numParticles;
    particles(i).pose = Xo_filt(1:3);               % robotX, robotY, theta
    particles(i).history = {};                      % empty cell array
    for l = 1:NLandmarks
        particles(i).landmarks(l).observed = false;
        particles(i).landmarks(l).mu = zeros(2,1);  % mjx, mjy
        particles(i).landmarks(l).sigma = zeros(2,2);
    end
end
% *********************************************************************
%
% End of FastSLAM Initialization.
%
% *********************************************************************



% Simulation iterations.
for idt = 2:(sim_num_iter+1)

    % PROCESS: Robot dynamics: X_t = f( X_t-1, u_t-1 )
    Xt_actual(:,idt) = slam_FofX( Xt_actual(:,idt-1), Ut .* vel_factor, rbt ) ...
                     +        noise_process(:,idt-1);

    % MEASUREMENT: Range finder: Y_t = g( X_t ).
    Yt(:,idt) = slam_GofX( Xt_actual(:,idt), Ut .* vel_factor, pfinder, all_constants ) ...
              +        noise_measure(:,idt);

    if( any( 1:2 == flag_live_plot ) )
        % Plots.
        v_bearing_rad_rel = Yt(1:2:end,idt);
        v_range           = Yt(2:2:end,idt);
        rbt = rbt.set_state( Xt_actual(1:3,idt) );
        pause(0.25);
        plot_scene_points( f1, f2, [all_points, all_constants], rbt, pfinder, v_bearing_rad_rel, v_range );
    end

    % --------------
    % FILTERS:
    % --------------
    % EKF:
    tic;
    % INSERT EKF HERE.
    % INSERT EKF HERE.
    % INSERT EKF HERE.
    % INSERT EKF HERE.
    flop_time_ekf = flop_time_ekf + toc;
    Kgain = zeros(num_states_rbt, num_states_obst); % TODO REMOVE.
    sig_ekf = zeros(size(sig_ekf)); % TODO REMOVE.
    [~,ss] = svd( Kgain );
    Ks_ekf(:,idt) = sum( diag( ss ) );
    Kp_ekf(:,idt) = prod( diag( ss ) );
    [~,ss] = svd( sig_ekf(:,:,idt) );
    Ss_ekf(:,idt) = sum( diag( ss ) );
    Sp_ekf(:,idt) = prod( diag( ss ) );


    % UKF:
    tic;
    [sig_ukf(:,:,idt), Xt_ukf(:,idt), Kgain, sig_predict_ukf(:,:,idt), mu_predict_ukf(:,idt)] = ...
            unscented_kalman_filter( sig_ukf(:,:,idt-1), Xt_ukf(:,idt-1), ...
                                      @slam_FofX, args_FofX, ...
                                      @slam_GofX, args_GofX, ...
                                      Qmat, Rmat, Ut, Yt(:,idt), ukf_lambda );
    flop_time_ukf = flop_time_ukf + toc;
    [~,ss] = svd( Kgain );
    Ks_ukf(:,idt) = sum( diag( ss ) );
    Kp_ukf(:,idt) = prod( diag( ss ) );
    [~,ss] = svd( sig_ukf(:,:,idt) );
    Ss_ukf(:,idt) = sum( diag( ss ) );
    Sp_ukf(:,idt) = prod( diag( ss ) );


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

    flop_time_fast = flop_time_fast + toc;

    if( 3 == flag_live_plot )
        %
        % Generate visualization plots of the current state of the filter
        %
        pauseTime = 0.1;
        plot_state(particles, map, pauseTime);
    end

    tic;

    Xt_fast(:,idt) = getBestState(particles, NLandmarks);      % Save the best robot state

    % Resample the particle set
    particles    = resample(particles);

    flop_time_fast = flop_time_fast + toc;

    % *********************************************************************
    %
    % End of FastSLAM
    %
    % *********************************************************************


    % *********************************************************************
    %
    % Particle Filter SLAM: Manoj Rajagopalan, 2020/06/06
    %
    % *********************************************************************
    

    if enable_pf
        tic;
        Qmat_pf = 0.49 * eye(num_states);
        Qmat_pf_sqrt = 0.7 * eye(num_states);

        for ip = 1 : num_particles
            Xt_particles(:,ip) = slam_FofX(Xt_particles(:,ip), Ut, rbt);
            % Artificially add noise to map terms so that cov matrix is
            % non-singular. Pristine Qmat only has non-zero diags for robot,
            % not map.
            pred_noise = aa273_mvnrnd(zeros(num_states,1), Qmat_pf_sqrt, 1);
            w_pred = aa273_mvnpdf(pred_noise, zeros(size(pred_noise)), Qmat_pf);
            Xt_particles(:,ip) = Xt_particles(:,ip) + pred_noise;
            g_particle = slam_GofX(Xt_particles(:,ip), Ut, pfinder, all_constants);
            w_meas = aa273_mvnpdf(Yt(:,idt), g_particle, Rmat);
            Wt_particles(ip) = w_meas; % * Wt_particles(ip); % unnormalized
        end
        Wt_particles = Wt_particles / sum(Wt_particles); % normalize
        Wt_particles_cum = cumsum(Wt_particles);
        new_sample_w = 1.0/num_particles * (rand() + (0:(num_particles-1)));
        [~, bins] = histc(new_sample_w, Wt_particles_cum);
        Xt_particles = Xt_particles(:,1+bins);
        Wt_particles = Wt_particles / sum(Wt_particles); % normalize

        % Analytics
        flop_time_pf = flop_time_pf + toc;
        [Xt_pf(:,idt), sig_pf] = aa273_mean_and_cov(Xt_particles, Wt_particles);
        pf_det_sigma(idt) = det(sig_pf);
        pf_convergence(:,idt) = convergence(Yt(:,idt), ...
                                            slam_GofX(Xt_pf(:,idt), Ut, pfinder, all_constants));
    end
end

% Smoothing.
Xt_eks = kalman_smoothing( mu_predict_ekf, sig_predict_ekf, ...
                           Xt_ekf        , sig_ekf, ...
                           Ut*ones(1,sim_num_iter+1), rbt, sim_num_iter);
Xt_uks = kalman_smoothing( mu_predict_ukf, sig_predict_ukf, ...
                           Xt_ukf        , sig_ukf, ...
                           Ut*ones(1,sim_num_iter+1), rbt, sim_num_iter);

fprintf( 'Computations times (cpu seconds per time step) are as follows:\n' );
fprintf( 'EKF : %f\n', flop_time_ekf  / sim_num_iter );
fprintf( 'UKF : %f\n', flop_time_ukf  / sim_num_iter );
fprintf( 'Fast: %f\n', flop_time_fast / sim_num_iter );
fprintf( 'PF  : %f\n', flop_time_pf   / sim_num_iter );

% Relative feature-to-robot measurements.
for idt = 1:size(Xt_actual,2)
    Yt_ekf (:,idt) = slam_GofX( Xt_ekf (:,idt), Ut, pfinder, all_constants );
    Yt_eks (:,idt) = slam_GofX( Xt_eks (:,idt), Ut, pfinder, all_constants );
    Yt_ukf (:,idt) = slam_GofX( Xt_ukf (:,idt), Ut, pfinder, all_constants );
    Yt_uks (:,idt) = slam_GofX( Xt_uks (:,idt), Ut, pfinder, all_constants );
    Yt_fast(:,idt) = slam_GofX( Xt_fast(:,idt), Ut, pfinder, all_constants );
    Yt_pf  (:,idt) = slam_GofX( Xt_pf  (:,idt), Ut, pfinder, all_constants );
end
dYt_ekf  = Yt - Yt_ekf;
dYt_eks  = Yt - Yt_eks;
dYt_ukf  = Yt - Yt_ukf;
dYt_uks  = Yt - Yt_uks;
dYt_fast = Yt - Yt_fast;
dYt_pf   = Yt - Yt_pf;

% Composite numerical results.
fprintf( 'Error scores:\n' );
error_scores.ekf  = composite_algo_error( Xt_actual - Xt_ekf , dYt_ekf, 'EKF' );
error_scores.eks  = composite_algo_error( Xt_actual - Xt_eks , dYt_eks, 'EKS' );
error_scores.ukf  = composite_algo_error( Xt_actual - Xt_ukf , dYt_ukf, 'UKF' );
error_scores.uks  = composite_algo_error( Xt_actual - Xt_uks , dYt_uks, 'UKS' );
error_scores.fast = composite_algo_error( Xt_actual - Xt_fast, dYt_fast, 'Fast' );
error_scores.pf   = composite_algo_error( Xt_actual - Xt_pf  , dYt_pf, 'PF' );

% Final result plots.
enable_plot_fast        = 0;
enable_plot_states      = 0;
enable_plot_measures    = 0;
enable_plot_map_no_time = 0;
enable_plot_metrics     = 0;

plot_results;

if( enable_pf )
    plot_convergence(sim_dt, ...
                     pf_convergence );
end

end
