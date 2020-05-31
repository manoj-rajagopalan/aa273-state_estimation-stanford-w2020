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


        % Generate range and relative bearing angles to obstacle points.
        % INPUTS:
        %    pt_obst_gbl     - 2xP  location of obstacle features in environment frame.
        %    pt_sens_gbl     - 2x1  location of point finder in environment frame.
        %    bearing_rad_ref - 1x1  reference bearing angle (radians): (abs = rel + ref).
        %
        % OUTPUTS:
        %    v_bearing_rad_rel - 1xP  array of relative bearing angles (radians) identified by point finder.
        %    v_range           - 1xP  array of positive distances identified by point finder.
        %
        % Verified 2020-05-31.
        function [v_bearing_rad_rel, v_range] = range_map( obj, pt_obst_gbl, pt_sens_gbl, bearing_rad_ref )
            num_obst = size( pt_obst_gbl, 2 );
            delta_pts = pt_obst_gbl - pt_sens_gbl * ones(1, num_obst);
            v_range = sqrt( sum( delta_pts .^ 2, 1 ) );
            v_bearing_rad_rel = atan2( delta_pts(2,:), delta_pts(1,:) ) - bearing_rad_ref;
        end


        % Plot range data and bearing angles in polar coordinates.
        % Verified 2020-05-31.
        function plot_range_map( obj, v_bearing_rad_rel, v_range )
            polar( v_bearing_rad_rel, v_range, 'o ' );
        end

    end

end
