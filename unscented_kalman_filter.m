%
% PURPOSE:
%    Unscented Kalman filter for discrete time system.
%    Executes two stages:
%    1. Predict.
%    2. Update.
%
% USAGE:
%    [sig, mu] = unscented_kalman_filter( sig, mu, @FofX, args_FofX, @GofX, args_GofX, Qmat, Rmat, Uvec, Yvec, lambda )
%
% INPUTS:
%    sig       - N x N: Coviance matrix of states at t=0.
%    mu        - N x 1: Mean vector of states at t=0.
%    FofX      - function pointer in the form:  X_t+1 = FofX( X_t, Uvec )
%    args_FofX - cell array of additional parameters needed for FofX().
%    GofX      - function pointer in the form:  Y_t   = GofX( X_t, Uvec )
%    args_GofX - cell array of additional parameters needed for FofX().
%    Qmat      - N x N: Covariance of process noise (dynamics equation). Assumed zero-mean.
%    Rmat      - R x R: Covariance of measurement noise (measurement equation). Assumed zero-mean.
%    Uvec      - P x 1: inputs.
%    Yvec      - R x 1: measurements.
%    lambda    - 1 x 1: parameter for adjusting to nonlinearities.
%
% OUTPUTS:
%    sig  - N x N: Covariance matrix of states.
%    mu   - N x 1: Mean vector of states.
%
% by Arjang Hourtash

function [sig, mu] = unscented_kalman_filter( sig, mu, FofX, args_FofX, GofX, args_GofX, Qmat, Rmat, Uvec, Yvec, lambda )

num_measure = length( Yvec );

% 1. Predict using sigma-points.
[x_sig_pts, weights] = unscented_transform( sig, mu, lambda );
num_weights = length( weights );
x_sig_pts_predict = NaN(size(x_sig_pts));
for idx = 1:num_weights
    x_sig_pts_predict(:,idx) = FofX( x_sig_pts(:,idx), Uvec, args_FofX{:} );
end

% 2. Update: using sigma-points.
y_sig_pts_predict = NaN(num_measure, num_weights);
for idx = 1:num_weights
    y_sig_pts_predict(:,idx) = GofX( x_sig_pts_predict(:,idx), Uvec, args_GofX{:} );
end

% 2. Update: Apply inverse UT.
[xx_sig_predict, ...
 xy_sig_predict, ...
 yy_sig_predict, ...
 x_mu_predict, ...
 y_mu_predict] = inverse_unscented_joint_transform( x_sig_pts_predict, ...
                                                    y_sig_pts_predict, weights );

% 2. Update: Add noise covariances.
xx_sig_predict = xx_sig_predict + Qmat;
yy_sig_predict = yy_sig_predict + Rmat;

% 2. Update: Apply Gaussian estimation.
Kgain = xy_sig_predict * inv( yy_sig_predict );
mu  = x_mu_predict   + Kgain * (Yvec - y_mu_predict);
sig = xx_sig_predict - Kgain * ( xy_sig_predict' );

end


% pts = UT( sig, mu )
function [sig_pts, weights] = unscented_transform( sig, mu, lambda )

num_states = length( mu );
den = lambda + num_states;
pt_sqrt = sqrt( den ) * sqrtm( sig );

sig_pts = mu*ones(1, 1+2*num_states) ...
        + [zeros(num_states,1), pt_sqrt, -pt_sqrt];
weights = [lambda, 0.5*ones(1, 2*num_states)]/den;

end


% Not strictly used, but left here as a reference.
% Since we need (a) xx_sig, (b) xy_sig, and (c) yy_sig, it is more
% efficient to combine all calculations in a single function.
function [sig, mu] = inverse_unscented_transform( sig_pts, weights )

num_states = size( sig_pts, 1 );
mu  = sum( sig_pts .* (ones(num_states, 1) * weights), 2 );
pts_from_mu = sig_pts - mu * ones(size(weights));
sig = zeros(num_states, num_states);
for idx = 1:length( weights )
    sig = sig + weights(idx) * (pts_from_mu(:,idx) * (pts_from_mu(:,idx))');
end

end


% [sig, mu] = inv_UT( pts )
function [xx_sig, xy_sig, yy_sig, x_mu, y_mu] = inverse_unscented_joint_transform( x_sig_pts, y_sig_pts, weights )

num_states  = size( x_sig_pts, 1 );
num_measure = size( y_sig_pts, 1 );

x_mu  = sum( x_sig_pts .* (ones(num_states , 1) * weights), 2 );
y_mu  = sum( y_sig_pts .* (ones(num_measure, 1) * weights), 2 );

onew = ones(size(weights));
x_from_mu = x_sig_pts - x_mu * onew;
y_from_mu = y_sig_pts - y_mu * onew;

xx_sig = zeros(num_states , num_states);
xy_sig = zeros(num_states , num_measure);
yy_sig = zeros(num_measure, num_measure);
for idx = 1:length( weights )
    xx_sig = xx_sig + weights(idx) * (x_from_mu(:,idx) * (x_from_mu(:,idx))');
    xy_sig = xy_sig + weights(idx) * (x_from_mu(:,idx) * (y_from_mu(:,idx))');
    yy_sig = yy_sig + weights(idx) * (y_from_mu(:,idx) * (y_from_mu(:,idx))');
end

end

