%
% PURPOSE:
%    Class definition for creating, manipulating, and drawing 2D polygons.
%    The polygons should be convex for simulating range/bearing sensing.
%
% USAGE:
%    obj = polygon( pts_arg, rot_ccw_rad, trans_2d )
%    obj.plot( fmt_string )
%
% INPUTS:
%    pts_arg        - 2xN array of vertices defining the polygon, in local polygon frame.
%    ---
%    The following rotation and translation transform the polygon from its local frame to
%    the environment's frame.
%
%    rot_ccw_rad    - Rotation to be applied about local [0,0] point, CCW=(+), radians.
%    trans_2d       - 1x2 or 2x1 translation to be applied.
%    ---
%    fmt_string     - format string used by "plot()".
%                     E.g.:  'b.-'
%
% by Arjang Hourtash
% 2020-05-23
%

classdef polygon

    properties
        % The following are 3xN list of N 2D points. The 3rd row is always 1s
        % to indicate that rows are "fixed" vectors.
        pts_2d_lcl    = zeros(3, 1); % in local polygon frame.
        pts_2d_gbl    = zeros(3, 1); % in environment frame.
        transform_3x3 = eye(3); % 3x3: Homogenous transform matrix in 2D.
    end

    methods

        % Constructor.
        % Verified 2020-05-24.
        function obj = polygon( pts_arg, rot_ccw_rad, trans_2d )
            obj = obj.set_pts( pts_arg, 'local' );
            obj = obj.set_transform( rot_ccw_rad, trans_2d );
            obj = obj.apply_transform();
        end


        % Verified 2020-05-24.
        function obj = set_pts( obj, pts_arg, choice )
            switch( choice )
                case 'local'
                    obj.pts_2d_lcl = obj.add_ones_row( pts_arg );
                case 'global'
                    obj.pts_2d_gbl = obj.add_ones_row( pts_arg );
            end
        end


        % Get the array of points.
        % INPUTS:
        %    choice   - 'local'  for points in local polygon frame.
        %               'global' for points in global environment's frame.
        %    flag_3xN - flag: 0=2xN, 1=3xN outputs.
        %
        % OUTPUTS:
        %    pts_arg - output array.
        %
        % Verified 2020-05-24.
        function pts_arg = get_pts( obj, choice, flag_3xN )
            switch( choice )
                case 'local'
                    pts_arg = obj.pts_2d_lcl;
                case 'global'
                    pts_arg = obj.pts_2d_gbl;
            end
            if( ~flag_3xN )
                pts_arg = pts_arg(1:2,:);
            end
        end


        % Convert desired rotation and translation into a 2D (3x3) version
        % of a homogenous transformation matrix, and save it to member variable.
        % Verified 2020-05-23.
        function obj = set_transform( obj, rot_ccw_rad, trans_2d )
            cs = cos( rot_ccw_rad );
            sn = sin( rot_ccw_rad );
            transform_arg = [cs, -sn, trans_2d(1)
                             sn,  cs, trans_2d(2)
                              0,   0,      1 ];
            obj.transform_3x3 = transform_arg;
        end


        % Verified 2020-05-23.
        function transform_arg = get_transform( obj )
            transform_arg = obj.transform_3x3;
        end


        % Add a "fixed vector" 3rd column of ones to the points array.
        % Verified 2020-05-24.
        function ThreeXn = add_ones_row( obj, TwoXn )
            ThreeXn = [TwoXn; ones(1, size(TwoXn,2))];
        end


        % Apply 2D transformation to local points.
        % Save result in global points member variable.
        % Verified 2020-05-24.
        function obj = apply_transform( obj )
            obj.pts_2d_gbl = obj.transform_3x3 * obj.pts_2d_lcl;
        end


        % Plot the global version of the polygon, closing the vertices.
        % Verified 2020-05-24.
        function plot( obj, fmt_string )

            % Default plotting format.
            if( ~exist( 'fmt_string', 'var' ) || isempty( fmt_string ) )
                fmt_string = '.-';
            end

            % Close the circuit.
            plot( obj.pts_2d_gbl(1,[1:end,1])', obj.pts_2d_gbl(2,[1:end,1])', fmt_string );

            % Touchup.
            daspect([1 1 1]);
            xlabel( 'X' );
            ylabel( 'Y' );
            grid on;
        end

    end

end
