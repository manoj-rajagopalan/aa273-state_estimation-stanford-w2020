%
% Resample the set of particles
%
function newParticles = resample(particles)

global Beta;         % resample threadhold for ESS [0 1]

numParticles = length(particles);

w = [particles.weight];

% normalize the weight
w = w/sum(w);


% Resampling?
ESS_before = ESS(w);

if (ESS_before < Beta)
    % Resampling
    newParticles = struct('weight',[], 'pose',[], 'history', [], 'landmarks', []);
    % Find the resampling index by weights
    [index] = resampleMultinomial(w);
    
    % Duplicate the particles
    for i = 1:numParticles
        idx = index(i);
        newParticles(i) = particles(idx);
        % After resmapling, set equal weights
        newParticles(i).weight = 1/numParticles;
    end
    
else
    % Not resampling
    newParticles = particles;
    for i = 1:numParticles
        newParticles(i).weight = w(i);
    end
    
end


end













