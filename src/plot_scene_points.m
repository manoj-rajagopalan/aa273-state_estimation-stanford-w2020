function plot_scene_points( f1, f2, all_points, rbt, pfinder, v_bearing_rad_rel, v_range )

% Draw the environment.
if( ~isempty( f1 ) )
    figure( f1 );
    plot( all_points(1,:)', all_points(2,:)', 'o ' )
    hold on
    rbt.plot();
    hold off
    ax = axis;
    ax(1) = min( ax(1), -0.1 );
    ax(2) = max( ax(2),  1.1 );
    ax(3) = min( ax(3), -0.1 );
    ax(4) = max( ax(4),  1.1 );
    axis( ax );
    title( 'Test Environment' );
end

if( ~isempty( f2 ) )
    figure( f2 );
    pfinder.plot_range_map( v_bearing_rad_rel, v_range )
    axis( [-1, 1, -1, 1] );
    title( 'Point Finder' );
end

end
