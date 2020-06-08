% Calculate the 1-norm of errors normalized by number of points,
% across time and multiple dimensions, if appropriate.
%
% The 1-norm across time is equivalent to integrating abs-of with respect to time, which
% is a more democratic representation compared to worst, mean, final values, which
% give preference to a particular choice.
%
% Also, the 1-norm, or the Manhattan distance, doesn't heavily emphasize outliers,
% like the 2-norm or the inf-norm does.
%
% Separately handle different flavors of variables.
%
function out = composite_algo_error( Xt__Xt_filt, Yt__Yt_filt, title_str )

num_pts = size(Xt__Xt_filt,2)-1;
ids = (2:(num_pts+1));
out.err_x_rbt_xy      = sum( sum( abs( Xt__Xt_filt(1:2,ids) ) ) ) / num_pts;
out.err_x_rbt_theta   = sum( sum( abs( Xt__Xt_filt(3,ids) ) ) ) / num_pts;
out.err_x_features_xy = sum( sum( abs( Xt__Xt_filt(4:end,ids) ) ) ) / num_pts;
out.err_y_bearing     = sum( sum( abs( Yt__Yt_filt(1:2:end,ids) ) ) ) / num_pts;
out.err_y_range       = sum( sum( abs( Yt__Yt_filt(2:2:end,ids) ) ) ) / num_pts;

fprintf( '%5s: rbt.xy = %g, rbt.theta = %g, pts.xy = %g, pts.bearing = %g, pts.range = %g\n', ...
    title_str, ...
    out.err_x_rbt_xy, ...
    out.err_x_rbt_theta, ...
    out.err_x_features_xy, ...
    out.err_y_bearing, ...
    out.err_y_range );

end
