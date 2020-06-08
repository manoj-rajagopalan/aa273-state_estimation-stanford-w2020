%
% Convert the measuremtn to the desired format
%
% Inputs:
% Yt - measurement at time t with the following format
%    - [bearing 1, range 1, bearing 2, range 2, beearing 3, range 3,
%       bearing 4, range 4]
%
% Outputs:
% yt - yt(i).id
%      yt(i).range
%      yt(i).bearing
%

function yt = convertYt(Yt)

for i = 1:(length(Yt)/2)
    id = i;
    range_i   = Yt(2*i);
    bearing_i = Yt(2*i - 1);
    
    yt(i).id = id;
    yt(i).range = range_i;
    yt(i).bearing = normalize_angle(bearing_i);
end


end