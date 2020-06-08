%
% Plot the final state
%
function plot_FinalState(Xactual, particles, landmarks)

figure(2)
hold on
grid on

L = struct2cell(landmarks);
p1 = plot( cell2mat(L(2,:)), cell2mat(L(3,:)),  'r+', 'MarkerSize',15, 'MarkerEdgeColor','r', 'MarkerFaceColor', 'r');

% determine the currently best particle
[bestWeight, bestParticleIdx] = max([particles.weight]);


% True robot path
robot_true_x = Xactual(1,:);
robot_true_y = Xactual(2,:);
p2 = plot(robot_true_x, robot_true_y, '-b', 'LineWidth', 2);

% get the history of the best particle
H = particles(bestParticleIdx).history;
H = cell2mat(H);
robot_path_x = H(1,:);
robot_path_y = H(2,:);
p3 = plot(robot_path_x, robot_path_y, '-r', 'LineWidth', 2);



% Plot the landmarks for the best particle
for i = 1:length(particles(bestParticleIdx).landmarks)
    if (particles(bestParticleIdx).landmarks(i).observed)
        l = particles(bestParticleIdx).landmarks(i).mu;
        p4 = plot(l(1), l(2), 'go', 'MarkerEdgeColor','g', 'MarkerFaceColor', 'g');
    end
end

ldg = legend([p1,p2,p3,p4], 'Landmarks - Actual', 'Robot Path - Actual', 'FastSLAM - Robot Path', 'Landmarks - FastSLAM');
%ldg.FontSize = 15;
title('FastSLAM', 'FontSize', 15)
xlabel('X');
ylabel('Y');
xlim([0 1]);
ylim([0 1]);









