%
% Generate random Normal distribution data points
% with specified mean and covariance
%
% J.D. Lee, 2020/5/11
%

% Inputs
% xbar - target mean
% covar - target covariance
% NP - number of points to be generated

% Outputs
% Y ~ N(mean, covariance)

function Y = genNDNormal(xbar, covar, NP)

ND = max(size(xbar));

X = randn([ND, NP]);

A = chol(covar, 'lower');   % A = cholesky decomposition
B = xbar;

Y = A*X + B;

