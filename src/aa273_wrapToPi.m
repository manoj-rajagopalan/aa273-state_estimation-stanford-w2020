% PURPOSE: provide functionality like MATLAB's wrapToPi (which requires
% Mapping toolbox)

function wrapped_theta = aa273_wrapToPi(theta)

two_pi = 2 * pi;
theta_mod_2pi = mod(theta, two_pi);
if theta_mod_2pi <= pi
    wrapped_theta = theta_mod_2pi;
else
    wrapped_theta = -(two_pi - theta_mod_2pi);
end

end
