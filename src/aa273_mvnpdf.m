%
% PURPOSE:
%    Compute probability density, like mvnpdf.
%    MATLAB's mvnpdf was found to be returning zeros so we wrote this.
%    Note that the inputs and output are transpose w.r.t. mvnpdf.
%
% USAGE:
%    mat = aa273_mvnpdf( X, mu, sigma )
%
% INPUTS:
%    X     - d x n matrix whose columns are d-dimensional samples
%    mu    - d x 1 mean-vector
%    sigma -  d x d covariance matrix
%    n     - number of random vectors
%
% OUTPUTS:
%    p  - 1 x n vector of probability densities
%
% See also mvnpdf.m
% by Manoj Rajagopalan

function p = aa273_mvnpdf(X, mu, sigma)
[d, n] = size(X);
x_minus_mu = X - repmat(mu,1,n);
sigma_inv_x_minus_mu = linsolve(sigma, x_minus_mu);
num = zeros(1,n);
for i = 1 : n
    num(i) = exp( -0.5 * x_minus_mu(:,i)' * sigma_inv_x_minus_mu(:,i) );
end
denom_sqr = (2 * pi)^numel(d) * det(sigma);
p = num / sqrt(denom_sqr);
end
