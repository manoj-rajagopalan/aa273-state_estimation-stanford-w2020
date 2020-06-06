%
% PURPOSE:
%    Jacobian for state transition for simultaneous localization and mapping.
%
% USAGE:
%    dF_by_dX_t = slam_dFofX_by_dX( X_t, U_t, rbt )
%
% INPUTS:
%    X_t       - Nx1 state vector at time t.
%    U_t       - Mx1 input vector at time t.
%    rbt       - instance of "robot.m" class.
%
% OUTPUTS:
%    dF_by_dX_t - NxN Jacobian of state, at time t.
%
% by Manoj Rajagopalan

function dF_by_dX_t = slam_dFofX_by_dX( X_t, U_t, rbt )

state_rbt = X_t(1:3);
state_env = X_t(4:end);
rbt = rbt.set_state( state_rbt );

dF_by_dX_t = zeros(numel(X_t));
dF_by_dX_t(1:3,1:3) = rbt.state_jacobian( U_t );
dF_by_dX_t(4:end, 4:end) = eye(numel(state_env));

end
