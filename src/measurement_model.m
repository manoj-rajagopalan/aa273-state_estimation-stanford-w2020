%
% Compute the expected measurement for a given robot pose and landmark
% Compute the Jacobian, Ct, of the measurement model
%
% Inputs:
% particle - current particle
% yt - current sensor measurement
%
% Outputs:
% y_hat - expected measurement
% Ct - Jacobian of measurement model
%

function [y_hat, Ct] = measurement_model(particle, yt)

% extract the id of the landmark
landmarkId = yt.id;

% extract the landmark (X,Y) position
landmarkPos = particle.landmarks(landmarkId).mu;
landmarkX   = landmarkPos(1);
landmarkY   = landmarkPos(2);

% extract the robot (X,Y) position
robotX = particle.pose(1);
robotY = particle.pose(2);
robotTheta = particle.pose(3);

% compute expected measurement
expectedRange = sqrt((landmarkX - robotX)^2 + (landmarkY - robotY)^2);
expectedBearing = normalize_angle( atan2(landmarkY-robotY , landmarkX-robotX) - robotTheta );
y_hat = [expectedRange; expectedBearing];

% Compute the Jacobian of the measurement model: Ct
Ct = zeros(2,2);
Ct(1,1) =  (landmarkX - robotX)/expectedRange;
Ct(1,2) =  (landmarkY - robotY)/expectedRange;
Ct(2,1) = -(landmarkY - robotY)/expectedRange^2;
Ct(2,2) =  (landmarkX - robotX)/expectedRange^2;

end




















