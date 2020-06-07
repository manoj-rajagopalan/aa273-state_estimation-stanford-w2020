%
% Convert points to landmarks format
% map(i).id
% map(i).x
% map(i).y
%
% Inputs:
% all_points - [x,y] location of the landmarks
%
% Outputs:
% map - contains
% id - landmark id
% x  - landmark x
% y  - landmark y

function map = convertPoints2Map(all_points)

% number of landmarks
NLandmarks = size(all_points,2);

for j = 1:NLandmarks
    % extract x and y coordinate of landmarks
    x = all_points(1,j);
    y = all_points(2,j);
    map(j).id = j;
    map(j).x  = x;
    map(j).y  = y;
end

