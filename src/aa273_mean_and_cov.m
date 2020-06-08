% 
% PURPOSE:
%     Compute mean and covariance of random samples, given their weights,
%     in a computationally efficient way (making a single pass through the
%     data instead of two).
%
% INPUTS:
%     X - d x n matrix whose columns are random samples
%     w - 1 x n row vector of weights (must sum to 1)

function [mean, cov] = aa273_mean_and_cov(X, w)
    [d, n] = size(X);
    mean = X * w;
    cov = zeros(d,d);
    for i = 1 : n
        x_minus_mean = X(:,i) - mean;
        cov = cov + w(i) * x_minus_mean * x_minus_mean';
    end
end

% function [mean, cov] = aa273_mean_and_cov(X, w)
%     [d, n] = size(X);
%     mean = zeros(d, 1);
%     outer_prod_expectation = zeros(d,d);
%     for i = 1 : n
%         mean = mean + w(i) * X(:,i);
%         outer_prod_expectation = outer_prod_expectation ...
%                                + w(i) * X(:,i) * X(:,i)';
%     end
%     
%     cov = outer_prod_expectation - mean * mean';
% end
