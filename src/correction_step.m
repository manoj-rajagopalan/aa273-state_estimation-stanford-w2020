%
% Particle correction step
%
% Inputs:
% particles - particles
% yt - current measurement containing the landmark observations
%      Each observation of yt(j) has:
%                          yt(j).id - observed landmark id
%                          yt(j).range - observed range
%                          yt(j).bearing - observed bearing
%
% The  particles(i).landmarks(l) indicate if this landmark has been
% observed before
%
% Outputs:
% 
%


function particles = correction_step(particles, yt)

global numParticles;    % number of particles
global R;               % measurement noise for both range and bearing

% Number of measurements in this time step
m = size(yt,2);

% Sensor noise covariance matrix Rt (2x2)
% this is called Qt in Prob. Robotis
factor = 1.0;
Rt = factor* R;

% loop over all particles
for i = 1:numParticles
    
    % current robot pose of particle i
    robot = particles(i).pose;    % robotX, robotY, robotTheta
    
    % process each measurement
    for j = 1:m
        % Get the id of the landmark corresponding to the j-th observation
        % particles(i).landmarks(l) is the EKF for this landmar
        l = yt(j).id;     % l = the landmark id of j-th observation
        
        % The (2x2) EKF of the landmark is given by its
        % mean: particles(i).landmarks(l).mu
        % covariance: particles(i).landmarks(l).sigma
        
        % If this landmark is observed for the first time: (initialize its
        % mu and sigma
        if (particles(i).landmarks(l).observed == false)
            
            % (1) Initialize its position based on the measurement and current
            % robot pose:
            robotX = robot(1);
            robotY = robot(2);
            robotTheta = robot(3);
            range   = yt(j).range;
            bearing = yt(j).bearing;
            particles(i).landmarks(l).mu = [robotX + range*cos(bearing + robotTheta);...
                                            robotY + range*sin(bearing + robotTheta)];
            
            % (2) get the Jacobian with respect to the landmark position
            %     => compute [y_hat, Ct] using measurement model for current
            %     robot and landmark 
            [y_hat, Ct] = measurement_model(particles(i), yt(j));
           
            
            % (3) initialize the EKF for this landmark
            particles(i).landmarks(l).sigma = inv(Ct)*Rt*inv(Ct)' ;

            
            % (4) Indicate this landmark has been observed
            particles(i).landmarks(l).observed = true;
            
            % note: NO need to set the default importance weight here since
            % we initialize it in the main program
            
        else
            
            % (1) get expected measurement
            [expectedY, Ct] = measurement_model(particles(i), yt(j));
            
            % (2) compute the measurement covariance
            R1 = Ct * particles(i).landmarks(l).sigma * Ct' + Rt;
            
            % (3) compute Kalman gain
            K = particles(i).landmarks(l).sigma * Ct' * inv(R1);
            
            % (4) compute the error between the yt and expectedY (remember to normalize the angle)
            y_diff = [yt(j).range; yt(j).bearing] - expectedY;
            y_diff(2) = normalize_angle( y_diff(2) );
            
            % (5) update the mean and covariance of the EKF for this landmark
            particles(i).landmarks(l).mu = particles(i).landmarks(l).mu + K*y_diff;
            particles(i).landmarks(l).sigma = (eye(2) - K*Ct)*particles(i).landmarks(l).sigma;
            
            % (6) - compute the likelihood of this observation
            %     - multiply with the former weight to account for
            %     observaing several features in one time step
            particles(i).weight = particles(i).weight * 1/sqrt(det(2*pi*R1)) * exp(-1/2*y_diff' *inv(R1)* y_diff);
        end
        
    end % measurement loop
end % particle loop

end







