% Script to plot convergence profiles

function plot_convergence(dt, ...
                          rbt_loc_convergence, ...
                          rbt_theta_convergence, ...
                          map_convergence, ...
                          det_sigma)
sim_num_iter = numel(rbt_loc_convergence);
tvec = dt * (1 : sim_num_iter);

figure;


h1(1) = subplot(4,1,1);
plot(tvec, rbt_loc_convergence);
title('PF convergence');
xlabel('Sim time (s)');
ylabel('Robot path (m)');

h1(2) = subplot(4,1,2);
plot(tvec, rbt_theta_convergence * 180/pi);
ylabel('Robot orientation (deg)');

h1(3) = subplot(4,1,3);
plot(tvec, map_convergence);
ylabel('Map (m)');

h1(4) = subplot(4,1,4);
plot(tvec, det_sigma);
ylabel('|Sigma|');

linkaxes(h1, 'x')

end
