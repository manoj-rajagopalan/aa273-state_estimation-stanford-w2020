function plot_scene( f1, f2, all_polygons, rbt, rfinder, m_range, debug )

% Draw the environment.
figure( f1 );
for idx = 1:length( all_polygons )
    all_polygons(idx).plot();
    hold on
end
rbt.plot();
hold off
title( 'Test Environment' );

figure( f2 );
if( debug )
    rfinder.plot_range_map_all( m_range );
else
    rfinder.plot_range_map_min( m_range );
end
title( 'Range Finder' );

end
