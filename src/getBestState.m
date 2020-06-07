%
% Get the best state estimation 
%
function Xbest = getBestState(particles, NLandmarks)

% determine the currently best particle
[bestWeight, bestParticleIdx] = max([particles.weight]);

pose = particles(bestParticleIdx).pose;     % robot pose [x, y, theta];

% mu1 = particles(bestParticleIdx).landmarks(1).mu;
% mu2 = particles(bestParticleIdx).landmarks(2).mu;
% mu3 = particles(bestParticleIdx).landmarks(3).mu;
% mu4 = particles(bestParticleIdx).landmarks(4).mu;
% 
% Xbest = [pose; mu1; mu2; mu3; mu4];

Xbest(1:3,1) = pose;
for i = 1:NLandmarks
    mui = particles(bestParticleIdx).landmarks(i).mu;
    Xbest(2*i + 2) = mui(1);
    Xbest(2*i + 3) = mui(2);
end

end



