%
% Resample
%
% J.D. Lee, 2020/5/19
%
% Inputs
% w - normalized weight
%
% Outputs
% index
%

function [index] = resampleMultinomial( w )

M    = length(w);
Q    = cumsum(w);
Q(M) = 1;     

index = zeros(M,1);

for i = 1:M
    sample = rand(1,1);
    j = 1;
    while (Q(j) < sample)
        j = j + 1;
    end
    index(i) = j;
end




















