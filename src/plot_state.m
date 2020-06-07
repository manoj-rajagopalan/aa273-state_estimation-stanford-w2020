%
% Plot the state
%
function plot_state(particles, landmarks, pauseTime)


%clf;
hold on;
grid on;

L = struct2cell(landmarks);
plot( cell2mat(L(2,:)), cell2mat(L(3,:)), 'r+', 'MarkerSize',10, 'MarkerEdgeColor','r', 'MarkerFaceColor', 'r')
xlim([0 1]);
ylim([0 1]);
pause(pauseTime)

% plot the particles
ppos = [particles.pose];
plot(ppos(1,:), ppos(2,:), 'g.');

% determine the currently best particle
[bestWeight, bestParticleIdx] = max([particles.weight]);

% draw the best robot path and landmark locations
xrobot = particles(bestParticleIdx).pose(1);
yrobot = particles(bestParticleIdx).pose(2);
plot(xrobot, yrobot, 'r^');

% draw the landmark locations for the best estimation
for i = 1:length(particles(bestParticleIdx).landmarks)
    if (particles(bestParticleIdx).landmarks(i).observed)
        l = particles(bestParticleIdx).landmarks(i).mu;
        plot(l(1), l(2), 'bo');
    end
end

xlabel('X')
ylabel('Y')
title('FastSLAM', 'FontSize', 15)

end

