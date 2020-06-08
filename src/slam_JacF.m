%
% PURPOSE:
%    Jacobian for state transition for simulataneous localization and mapping.
%
% USAGE:
%    dF_dX = slam_JacF( X_t, U_t, rbt )
%
% INPUTS:
%    X_t       - Nx1 state vector at time t.
%    U_t       - Mx1 input vector at time t.
%    rbt       - instance of "robot.m" class.
%
% OUTPUTS:
%    dF_dX     - NxN state vector at time t+1.
%
% by Arjang Hourtash

function dF_dX = slam_JacF( X_t, U_t, rbt )

state_rbt = X_t(1:3);
rbt = rbt.set_state( state_rbt );

% Jacobian.
% Since obstacles are stationary, carry them to
% the next time step without modification. So their derivative is I.
dF_dX = eye( length( X_t ) );
dF_dX(1:3,1:3) = rbt.Jacobian_state_transition( U_t );

end
