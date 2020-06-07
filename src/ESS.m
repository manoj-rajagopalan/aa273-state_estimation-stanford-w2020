%
% Compute effective Sample Size
%
% J.D. Lee, 2020/5/20
%
% Inputs
% w - weight
%
% Outputs
% ess - effective sample size

function [ess] = ESS(w)

N = size(w,2);
cv = sum( ( w.*N - 1 ).^2 ) / N;
ess = 1 / ( 1 + cv );