%
% Particle filter prediction step
%
% Inputs:
% particles - particles
% u - control input
%
% Outputs:
% updated robot pose (x, y, theta) for each particle
%

function particles = prediction_step(particles, u)

global dT;       % simulation time step
global Q;        % process noise Q = 0.1*dT*eye(3)
global numParticles;

% Get the control inputs at current time step
V_t   = u(1);
Phi_t = u(2);

for i = 1:numParticles
    % append the old position to the history of particles
    particles(i).history{end+1} = particles(i).pose;
    
    xt    = particles(i).pose(1);    % robot x position
    yt    = particles(i).pose(2);    % robot y position
    theta = particles(i).pose(3);    % robot theta
    
    % generate the process random noise
    Wt = genNDNormal([0; 0; 0], Q, 1);
    
    % simulate the dynamic system with noise
    particles(i).pose(1) = xt + dT*V_t*cos(theta) + Wt(1);
    particles(i).pose(2) = yt + dT*V_t*sin(theta) + Wt(2);
    particles(i).pose(3) = theta + dT*Phi_t + Wt(3);
end

end















