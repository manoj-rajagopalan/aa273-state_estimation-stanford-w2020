% Extended Kalman Filter for discrete time simulation of a differential
% wheeled robot

function [sigma, mu, sigma_predict, mu_predict] = extended_kalman_filter(sigma, mu, Fx, Fxargs, Gx, Gxargs, Jac, Jacargs, Q, R, U, Y)
    %Predict step
    mu_predict = Fx(mu, U, Fxargs)
    A = Jac(mu, U, Jacargs)
    inverseA = inv(A)
    sigma_predict = A*sig*inverseA + Q
    
    %Update step variables
    C = Jac(Y, U, Jacargs)
    Ctrans = transpose(C)
    coeff = inv(C*sigma_predict*Ctrans + R)
    measure = Y - Gx(mu, U, Gxargs)
    
    %Updating mu
    mu = mu_predict + sigma_predict*Ctrans*coeff*measure
    sigma = sigma_predict - sigma_predict*Ctrans*coeff*sigma_predict*C
end
