%
% PURPOSE:
%    Unscented Kalman filter for discrete time system.
%    Executes two stages:
%    1. Predict.
%    2. Update.
%
% NOTE:
%    invUT( UT(*) ) returns the original inputs.
%    UT( invUT(*) ) does NOT return the original inputs.
%
% USAGE:
%    [sig, mu, Kgain, sig_predict, mu_predict] = unscented_kalman_filter( sig, mu, @FofX, args_FofX, @GofX, args_GofX, Qmat, Rmat, Uvec, Yvec, lambda )
%
% INPUTS:
%    sig       - N x N: Coviance matrix of states at t=0.
%    mu        - N x 1: Mean vector of states at t=0.
%
%    FofX      - function pointer in the form:  X_t+1 = FofX( X_t, Uvec, args_FofX{:} )
%    args_FofX - cell array of additional parameters needed for FofX().
%
%    GofX      - function pointer in the form:  Y_t   = GofX( X_t, Uvec, args_GofX{:} )
%    args_GofX - cell array of additional parameters needed for FofX().
%
%    Qmat      - N x N: Covariance of process noise (dynamics equation). Assumed zero-mean.
%    Rmat      - R x R: Covariance of measurement noise (measurement equation). Assumed zero-mean.
%    Uvec      - P x 1: inputs.
%    Yvec      - R x 1: measurements.
%    lambda    - 1 x 1: parameter for adjusting to nonlinearities.
%
% OUTPUTS:
%    sig   - N x N: Covariance matrix of states.
%    mu    - N x 1: Mean vector of states.
%    Kgain - N x R: Matrix multiplier of mean error.
%
%    Useful for smoothing:
%    sig_predict - N x N: Covariance matrix of states after prediction.
%    mu_predict  - N x 1: Mean vector of states after prediction.
%
% by Arjang Hourtash

function [sig, mu, Kgain, sig_predict, mu_predict] = unscented_kalman_filter( sig, mu, FofX, args_FofX, GofX, args_GofX, Qmat, Rmat, Uvec, Yvec, lambda )

% NOTE:
%    invUT( UT(*) ) returns the original inputs.
%    UT( invUT(*) ) does NOT return the original inputs.

% 1. Predict using sigma-points.
[x_sig_pts, weights] = unscented_transform( sig, mu, lambda );
num_weights = length( weights );
x_sig_pts_predict = zeros(size(x_sig_pts));
for idx = 1:num_weights
    x_sig_pts_predict(:,idx) = FofX( x_sig_pts(:,idx), Uvec, args_FofX{:} );
end
[sig_predict, mu_predict] = inverse_unscented_transform( x_sig_pts_predict, weights );
sig_predict = sig_predict + Qmat; % Add noise covariance.

% 2. Update: using sigma-points.
[x_sig_pts_update, ~] = unscented_transform( sig_predict, mu_predict, lambda );
y_sig_pts_update = zeros(length( Yvec ), num_weights);
for idx = 1:num_weights
    y_sig_pts_update(:,idx) = GofX( x_sig_pts_update(:,idx), Uvec, args_GofX{:} );
end

% 2. Update: Apply inverse UT.
[~, ...
 xy_sig_update, ...
 yy_sig_update, ...
 ~, ...
 y_mu_update] = inverse_unscented_joint_transform( x_sig_pts_update, ...
                                                   y_sig_pts_update, weights );

% 2. Update: Add noise covariance.
yy_sig_update = yy_sig_update + Rmat;

% 2. Update: Apply Gaussian estimation.
Kgain = xy_sig_update * inv( yy_sig_update );
mu    = mu_predict  + Kgain * (Yvec - y_mu_update);
sig   = sig_predict - Kgain * ( xy_sig_update' );

end


% [pts, wgts] = UT( sig, mu )
%    where:
%        pts = [mu, mu + sqrtm(den*sig), mu - sqrtm(den*sig)]
%        wgts = [lambda, 1/2*[1 1 ...]]/den
%        den  = n + lambda
%        n    = length( mu )
function [sig_pts, weights] = unscented_transform( sig, mu, lambda )

num_states = length( mu );
den = lambda + num_states;
pt_sqrt = sqrtmat( den * sig );

sig_pts = mu*ones(1, 1+2*num_states) ...
        + [zeros(num_states,1), pt_sqrt, -pt_sqrt];
weights = [lambda, 0.5*ones(1, 2*num_states)]/den;

end


% Use partial SVD or Cholesky decomposition based matrix square-root:
%
%    Amat = Asqrt * Asqrt'
%
% The SVD approach below is only applicable only to symmetric matrices.
function Asqrt = sqrtmat( Amat )

sqrt_choice = 2;
switch( sqrt_choice )
    case 1
        % sqrtm(A) = U sqrtm(S)
        % Resulting Asqrt ~= Asqrt'. So [Amat = Asqrt * Asqrt'] but [Amat ~= Asqrt * Asqrt].
        % This lines up the sigma points along the principal axes of the error ellipse.
        [uu, ss] = svd( Amat );
        ss_sqrt = diag( diag( ss ) .^ 0.5 );
        Asqrt = uu * ss_sqrt;
    case 2
        % sqrtm(A) = U sqrtm(S) * V'
        % Equivalent to sqrtm() in most cases. However, is more robust when rank deficient.
        % Result is symmetrical for symmetrical "Amat".
        % For "Amat" not symmetrical, indefinite, or semidefinite, "Asqrt" is real.
        [uu, ss, vv] = svd( Amat );
        ss_sqrt = diag( diag( ss ) .^ 0.5 );
        Asqrt = uu * ss_sqrt * vv';
    case 3
        % Result is symmetrical for symmetrical "Amat".
        % For "Amat" positive definite. If not, resulting matrix will be complex.
        Asqrt = sqrtm(Amat);
end

end


% Not strictly used, but left here as a reference.
% Since we need (a) xx_sig, (b) xy_sig, and (c) yy_sig, it is more
% efficient to combine all calculations in a single function.
%
% mu  = sum_i( weights(i) * sig_pts(:,i)
% sig = sum_i( weights(i) * ( sig_pts(:,i) - mu )*( ... )'
%
function [sig, mu] = inverse_unscented_transform( sig_pts, weights )

mu = sig_pts * (weights');
pts_from_mu = sig_pts - mu * ones(size(weights));
sig = pts_from_mu * diag(weights) * (pts_from_mu');

end


% [sig, mu] = inv_UT( pts )
% mu  = sum_i( weights(i) * sig_pts(:,i)
% sig = sum_i( weights(i) * ( sig_pts(:,i) - mu )*( ... )'
function [xx_sig, xy_sig, yy_sig, x_mu, y_mu] = inverse_unscented_joint_transform( x_sig_pts, y_sig_pts, weights )

x_mu  = x_sig_pts * (weights');
y_mu  = y_sig_pts * (weights');

onew = ones(size(weights));
x_from_mu = x_sig_pts - x_mu * onew;
y_from_mu = y_sig_pts - y_mu * onew;

dw = diag(weights);
xx_sig = x_from_mu * dw * (x_from_mu');
xy_sig = x_from_mu * dw * (y_from_mu');
yy_sig = y_from_mu * dw * (y_from_mu');

end


% Test to demonstrate:
%    invUT( UT(*) ) = (*)
function [sig_err, mu_err] = iut_ut( sig, mu, lambda )

[sig_pts, weights] = unscented_transform( sig, mu, lambda );
[sig_1, mu_1] = inverse_unscented_transform( sig_pts, weights );
sig_err = max(max(abs( sig - sig_1 )));
mu_err = max(abs( mu - mu_1 ));

end


% Test to refute:
%    UT( invUT(*) ) = (*)
function [sig_pt_err, weight_err] = ut_iut( sig_pts, weights, lambda )

[sig, mu] = inverse_unscented_transform( sig_pts, weights );
[sig_pts_1, weights_1] = unscented_transform( sig, mu, lambda );
weight_err = max(abs( weights - weights_1 ));
sig_pt_err = max(max(abs( sig_pts - sig_pts_1 )));

end
