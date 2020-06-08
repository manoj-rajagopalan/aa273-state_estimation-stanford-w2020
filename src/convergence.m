%
% PURPOSE:
%     Evaluate convergence properties given actual and modeled measurement
%     values. To be called with value of GofX(Xt_mean) for each approach.

function c = convergence(y, g)
ny = numel(y);
c = zeros(ny+2, 1);
c(1:ny) = abs( y - g);
c(end-1) = sum(c(1:2:ny)); % summarize bearings score
c(end) = sum(c(2:2:ny)); % summarize range score
end
