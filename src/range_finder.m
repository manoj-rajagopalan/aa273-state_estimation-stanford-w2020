%
% PURPOSE:
%    Class definition for the calculations of a range finder beam.
%
% USAGE:
%    obj = range_finder( res_arg )
%    m_range = obj.range_map( pygons, pt_gbl, bearing_rad_ref )
%    obj.plot_range_map_min( m_range )
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
% 2020-05-23
%

classdef range_finder

    properties
        resolution    = 1; % Number of bearing angles to consider.
        v_bearing_rad = 0; % Array of bearing angles (radians), relative to a reference.
    end

    methods

        % Constructor.
        % Verified 2020-05-24.
        function obj = range_finder( res_arg )
            obj = obj.set_resolution( res_arg );
        end


        % Verified 2020-05-24.
        function obj = set_resolution( obj, res_arg )
            % Set resolution.
            res_arg = max( 1, round( res_arg ) ); % Integers only. Values >= 1.
            obj.resolution = res_arg;

            % Set relative bearing angles.
            tmp = linspace( 0, 1, obj.resolution + 1 )'*(2*pi);
            obj.v_bearing_rad = tmp(1:(end-1)); % Last point is a duplicate, so exclude it.
        end


        % Verified 2020-05-24.
        function [res_arg, v_bearing_arg] = get_resolution( obj )
            res_arg       = obj.resolution;
            v_bearing_arg = obj.v_bearing_rad;
        end


        % Generate range data for a "beam" center location, and a vector of bearing angles.
        % INPUTS:
        %    pygons          - 1xP  array of polygons.
        %    pt_gbl          - 2x1  location of range finder in environment frame.
        %    bearing_rad_ref - 1x1  reference bearing angle (radians): (abs = rel + ref).
        %
        % OUTPUTS:
        %    m_range - MxP  array of positive distances identified by range finder.
        %
        % Verified 2020-05-24.
        function m_range = range_map( obj, pygons, pt_gbl, bearing_rad_ref )
            m_range = NaN(size(obj.v_bearing_rad,1), size(pygons,2)); % For array sizing.
            for idx = 1:length(pygons)
                for idy = 1:length(obj.v_bearing_rad)
                    m_range(idy,idx) = obj.range_from_pt_and_bearing( pygons(idx), pt_gbl, ...
                        bearing_rad_ref + obj.v_bearing_rad(idy) );
                end
            end
        end


        % Plot individual range maps separately.
        % Verified 2020-05-24.
        function plot_range_map_all( obj, m_range )
            for idx = 1:size(m_range,2)
                obj.plot_range_map_single( obj.v_bearing_rad, m_range(:,idx) );
                hold on
            end
            hold off
        end


        % Plot the minimum range of all maps, as a single map.
        % Verified 2020-05-24.
        function plot_range_map_min( obj, m_range )
            obj.plot_range_map_single( obj.v_bearing_rad, min( m_range, [], 2 ) );
        end


        % Plot range data and bearing angles in polar coordinates.
        % Verified 2020-05-24.
        function plot_range_map_single( obj, v_bearing_rad_arg, v_range )
            % Convert "inf" range values to "NaN" for plotting.
            v_range( isinf( v_range ) ) = NaN;
            polar( v_bearing_rad_arg, v_range, '. ' );
            grid on
            axis( [-1.2, 1.2, -1.2, 1.2] );
        end


        % Given a global point and an absolute bearing, find the distance
        % to "this" polygon.
        %
        % INPUTS:
        %    pygon           :      Polygon object to be considered.
        %    pt_gbl          : 2x1  Source point for range finder "beam".
        %    bearing_rad_abs : 1x1  Bearing of "beam" in environment frame (radians).
        %
        % OUTPUTS:
        %    rng : 1x1  Distance from "pt_gbl" to "this" polygon along "bearing_rad_abs".
        %               or "inf" if there's no "beam" contact.
        %
        % Verified 2020-05-24.
        function rng = range_from_pt_and_bearing( obj, pygon, pt_gbl, bearing_rad_abs )

            % Useful parameters.
            cs = cos( bearing_rad_abs );
            sn = sin( bearing_rad_abs );
            pts_poly_gbl = pygon.get_pts( 'global', 1 );

            % Create hyperplane tests to check for interference.
            % Hyperplane along ray direction, value=0 at "pt_gbl".
            v_along = [cs; sn; -[cs, sn] * pt_gbl];

            % Hyperplane perpendicular to ray, value=0 at "pt_gbl".
            v_perp = [-sn; cs; -[-sn, cs] * pt_gbl];

            % Check to see if range is positive for any of the vertices.
            if( any( v_along' * pts_poly_gbl >= 0 ) )

                % The range finder "beam" is pointing toward "this" polygon.
                % Check to see if there is a sign change between any set of indices.
                hyper_cut = v_perp' * pts_poly_gbl;
                if( ( min( hyper_cut ) <= 0 ) && ( max( hyper_cut ) >= 0 ) )

                    % The range finder "beam" is cutting "this" polygon.
                    % Find all consecutive pairs of vertices which are cut by "beam".
                    hyper_cut_p1 = hyper_cut([1:end,1]); % Close the circuit.
                    pts_poly_2d_circuit = pts_poly_gbl(1:2,[1:end,1]); % Close the circuit.
                    rng_possible = inf(size(hyper_cut));
                    for idx = 1:length( hyper_cut )
                        if( hyper_cut_p1(idx) * hyper_cut_p1(idx+1) <= 0 )

                            % Perpendicular hyperplane test indicates opposite signs
                            % for the (idx) and (idx+1) pair of vertices. The beam is
                            % is cutting here. Find the 2D intersection between this
                            % intervertex line segment and the beam.

                            % Line 1: pt_gbl + alpha * v_along(1:2)
                            % Line 2: pts_poly_2d_circuit(idx) + beta *
                            %        (pts_poly_2d_circuit(idx+1) - pts_poly_2d_circuit(idx))
                            % Line 1 = Line 2
                            % Intersection: Ax=b, x=[alpha; beta].
                            bvec = pts_poly_2d_circuit(:,idx) - pt_gbl;
                            Amat = [v_along(1:2), (pts_poly_2d_circuit(:,idx) - pts_poly_2d_circuit(:,idx+1))];
                            xvec = Amat \ bvec;

                            % pt_intersect = pt_gbl + xvec(1) * v_along(1:2);
                            % rng = norm( pt_gbl - pt_intersect ) = alpha
                            rng_possible(idx) = xvec(1);
                        end
                    end
                    rng = min( rng_possible(rng_possible >= 0) ); % Only forward pointing (positive) values.

                else
                    % The range finder "beam" is not cutting through "this" polygon.
                    rng = inf;
                end
            else
                % The range finder "beam" is pointing away from "this" polygon.
                rng = inf;
            end
        end
    end

end
