%
% PURPOSE:
%    Class definition for the calculations of a point finder.
%
% USAGE:
%    obj = point_finder( res_arg )
%    [v_bearing_rad_rel, v_range] = obj.range_map( pt_obst_gbl, pt_sens_gbl, bearing_rad_ref )
%    obj.plot_range_map( v_bearing_rad_rel, v_range )
%
% INPUTS:
%    res_arg          -  angular resolution, i.e. number of divisions per 360 deg.
%    pt_gbl           -  2x1 global coordinates of range finder.
%    bearing_rad_ref  -  1x1 orientation (radians) of range finder relative to the environment.
%
% OUTPUTS:
%    m_range  - (res_arg) x (num_polygons)  array of distances to the polygons.
%
% by Arjang Hourtash
% 2020-05-31
%

classdef point_finder

    methods

        % Constructor.
        % Verified 2020-05-31.
        function obj = point_finder()
        end


        % Generate relative bearing angles and range to obstacle points.
        % INPUTS:
        %    pt_obst_gbl     - 2xP  location of obstacle features in environment frame.
        %    pt_sens_gbl     - 2x1  location of point finder in environment frame.
        %    bearing_rad_ref - 1x1  reference bearing angle (radians): (abs = rel + ref).
        %
        % OUTPUTS:
        %    v_bearing_rad_rel - 1xP  array of relative bearing angles (radians) identified by point finder.
        %    v_range           - 1xP  array of positive distances identified by point finder.
        %
        % Verified 2020-06-06.
        function [v_bearing_rad_rel, v_range] = range_map( obj, pt_obst_gbl, pt_sens_gbl, bearing_rad_ref )
            num_obst = size( pt_obst_gbl, 2 );
            delta_pts = pt_obst_gbl - pt_sens_gbl * ones(1, num_obst);
            v_range = sqrt( sum( delta_pts .^ 2, 1 ) );
            v_bearing_rad_rel = atan2( delta_pts(2,:), delta_pts(1,:) ) - bearing_rad_ref;

            % Handle wrapping.
            v_bearing_rad_rel = mod( v_bearing_rad_rel + pi, 2*pi ) - pi;
        end


        % Generate partial derivative of [relative bearing, range] with respect to [sensor location, obstacle locations].
        % For derivation, see HW#6, Problem 3.
        %
        % INPUTS:
        %    Same as those for "range_map()".
        %
        % OUTPUTS:
        %    Jac_map - Jacobian:
        %              rows = pairs of [bearing_rad_rel
        %                               range          ]_1
        %                     pairs of [bearing_rad_rel
        %                               range          ]_2
        %                     ...
        %              cols = [pt_sens_gbl, bearing_rad_ref, Pxy_obst_1, Pxy_obst_2, ...]
        %              where M = number of 2x1 points in "pt_obst_gbl".
        %
        % Verified 2020-06-05.
        function Jac_map = Jacobian_range_map( obj, pt_obst_gbl, pt_sens_gbl, bearing_rad_ref )
            num_features = size(pt_obst_gbl, 2);
            Jac_map = zeros(2*num_features, 3+2*num_features);
            for idx = 1:num_features
                tmp = pt_obst_gbl(:,idx) - pt_sens_gbl;
                tmp2 = tmp' / norm(tmp);
                id_vert = (idx-1)*2;
                Jac_map(id_vert+1,1:2) = tmp2 * [0, -1
                                                 1,  0]; % d(bearing)/dPxy

                Jac_map(id_vert+2,1:2) = tmp2 * [-1,  0
                                                  0, -1]; % d(range)/dPxy

                Jac_map(id_vert+1,id_vert+(4:5)) = tmp2 * [0, 1
                                                          -1, 0]; % d(bearing)/d(m_i)

                Jac_map(id_vert+2,id_vert+(4:5)) = tmp2 * [1, 0
                                                           0, 1]; % d(range)/d(m_i)
            end
            Jac_map(1:2:end,3) = -1; % d(bearing)/d(theta)
        end


        % Plot range data and bearing angles in polar coordinates.
        % Verified 2020-05-31.
        function plot_range_map( obj, v_bearing_rad_rel, v_range )
            polar( v_bearing_rad_rel, v_range, 'o ' );
        end

    end

end
