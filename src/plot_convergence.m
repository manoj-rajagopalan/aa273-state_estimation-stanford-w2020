% Script to plot convergence profiles

function plot_convergence(dt, convergence, det_sigma)

sim_num_iter = size(convergence, 2);
tvec = dt * (1 : sim_num_iter);

figure;

ny = size(convergence, 1) - 2;

h(1) = subplot(4,1,1);
plot(tvec, convergence(1:2:ny, :)');
legend(string(1:4));
title('Bearings');
xlabel('Sim time (s)');
ylabel('Angle (deg)');

h(2) = subplot(4,1,2);
plot(tvec, convergence(end-1,:));
title('Bearings (sum)');
xlabel('Sim time (s)');
ylabel('Angle (deg)');

h(3) = subplot(4,1,3);
plot(tvec, convergence(2:2:ny,:));
legend(string(1:4));
title('Ranges');
xlabel('Sim time (s)');
ylabel('Distance (m)');

h(4) = subplot(4,1,4);
plot(tvec, convergence(end,:));
title('Ranges (sum)');
xlabel('Sim time (s)');
ylabel('Distance (m)');

linkaxes(h, 'x')

end
