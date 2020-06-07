%
% PURPOSE:
%    Generate vectors from non-standard normal distribution.
%
%    X = sigma^(1/2) * Z + mu
%
%        where Z = from standard normal distribution
%
% USAGE:
%    mat = aa273_mvnrnd( mu, sigma_sqrt, n )
%
% INPUTS:
%    mu    - mean vector (dx1)
%    sigma_sqrt - precomputed square-root of covariance matrix (d x d)
%    n     - number of random vectors
%
% OUTPUTS:
%    mat  - d x n matrix, where d is the dimension of mu vector
%    This is like mvnrnd() but the output is transposed.
%
% See also mvnrnd.m
% by Arjang Hourtash

function mat = aa273_mvnrnd( mu, sigma_sqrt, n )

% Z = standard normal vector.
% X = non-standard normal vector.
%
% Z = sigma^(-1/2) * (X - mu)
% X = sigma^(1/2) * Z + mu

d = size( mu, 1 );
mat_z = randn( d, n );
mat   = sigma_sqrt * mat_z + repmat(mu, 1 ,n );

end
