%Normalize phi to be between -pi and pi

function [phiNorm] = normalize_angle(phi)


while (phi>pi)
	phi = phi - 2*pi;
end

while(phi<-pi)
	phi = phi + 2*pi;
end

phiNorm = phi;

end