%
% PURPOSE:
%    State transition for simulataneous localization and mapping.
%
% USAGE:
%    X_tp1 = slam_FofX( X_t, U_t, rbt )
%
% INPUTS:
%    X_t       - Nx1 state vector at time t.
%    U_t       - Mx1 input vector at time t.
%    rbt       - instance of "robot.m" class.
%
% OUTPUTS:
%    X_tp1     - Nx1 state vector at time t+1.
%
% by Arjang Hourtash

function X_tp1 = slam_FofX( X_t, U_t, rbt )

state_rbt = X_t(1:3);
state_env = X_t(4:end);
rbt = rbt.set_state( state_rbt );
rbt = rbt.state_transition( U_t );

% Since obstacles are stationary, carry them to
% the next time step without modification.
X_tp1 = [rbt.get_state(); state_env];

end
