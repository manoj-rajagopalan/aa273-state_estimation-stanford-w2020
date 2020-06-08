%
% PURPOSE:
%    Class definition for moving 2D robot.
%
% USAGE:
%    obj = robot( sim_dt )
%
% INPUTS:
%
% by Arjang Hourtash
% 2020-05-24
%

classdef robot

    properties
        pygon_scale = 0.02;
        pygon       = [];

        % 3x1  [X-position; Y-position; orientation (radians)] in environment frame.
        state       = [0; 0; 0];
        sim_dt      = 0;
    end

    methods

        % Constructor.
        % Verified 2020-05-24.
        function obj = robot( sim_dt_arg )

            % Robot's polygonal shape.
            sqrt3o2 = sqrt(3)/2;
            arrow = [0   ,  0
                    -0.5 , -sqrt3o2
                     1   ,  0
                    -0.5 ,  sqrt3o2]';
            obj.pygon = polygon( arrow*obj.pygon_scale, 0, [0;0] );

            % State transition update time.
            obj.sim_dt = sim_dt_arg;
        end


        % Verified 2020-05-24.
        function obj = set_pose( obj, rot_arg, trans_arg )
            obj.state = [trans_arg(:); rot_arg];
        end


        % Verified 2020-05-24.
        function [rot_arg, trans_arg] = get_pose( obj )
            rot_arg   = obj.state(3);
            trans_arg = obj.state(1:2);
        end


        % Verified 2020-05-24.
        function obj = set_state( obj, x3 )
            obj.state = x3;
        end


        % Verified 2020-05-24.
        function x3 = get_state( obj )
            x3 = obj.state;
        end


        % State transition: from t(k) to t(k+1).
        %    / x_t+1 \                      / V cos(O_t) \
        %    | y_t+1 | = X_t+1 = X_t + dt * | V sin(O_t) |
        %    \ O_t+1 /                      \    omega   /
        %
        % INPUTS:
        %    vel_lin_ang - 2x1  input velocity vector: u = [linear; angular (radians)].
        %
        % Verified 2020-05-24.
        function obj = state_transition( obj, vel_lin_ang )
            cs_sn = [cos( obj.state(3) )
                     sin( obj.state(3) )];
            obj.state = obj.state + obj.sim_dt * [ vel_lin_ang(1) * cs_sn
                                                   vel_lin_ang(2) ];
        end


        % Generate partial derivative of [state transition] with respect to [states].
        %                         / 0  ,  0  ,  -sin(O_t) \
        %    Jac = I_3 + dt * V * | 0  ,  0  ,   cos(O_t) |
        %                         \ 0  ,  0  ,       0    /
        %
        % INPUTS:
        %    Same as those for "state_transition()".
        %
        % OUTPUTS:
        %    Jac_transition - Jacobian:
        %                     rows = [Pxy, orientation]
        %                     cols = [Pxy, orientation]
        %
        % Verified 2020-06-05.
        function Jac_transition = Jacobian_state_transition( obj, vel_lin_ang )
            Pxy = obj.state(1:2);
            th  = obj.state(3);
            Vt  = vel_lin_ang(1);
            phi = vel_lin_ang(2);
            Jac_transition = eye(size(obj.state,1));
            Jac_transition(1:2,3) = Jac_transition(1:2,3) + obj.sim_dt * Vt * [-sin(th);
                                                                                cos(th)];
        end


        % Verified 2020-05-24.
        function plot( obj, fmt_string )
            obj.pygon = obj.pygon.set_transform( obj.state(3), obj.state(1:2) );
            obj.pygon = obj.pygon.apply_transform();
            if( ~exist( 'fmt_string', 'var' ) )
                fmt_string = '';
            end
            obj.pygon.plot( fmt_string );
        end
    end

end
