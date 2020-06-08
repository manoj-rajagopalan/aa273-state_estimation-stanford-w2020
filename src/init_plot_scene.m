function [f1, f2] = init_plot_scene(flag_live_plot)

if( any( 1:2 == flag_live_plot ) )
    % Initialize 1st figure.
    f1 = figure;
    p1 = get( f1, 'Position' );
    p1(1) = 100;
    set( f1, 'Position', p1 );
else
    f1 = [];
end

if( 2 == flag_live_plot )
    % Initialize 2nd figure.
    f2 = figure;
    p2 = get( f2, 'Position' );
    p2(1) = p1(1) + p1(3) + 20;
    set( f2, 'Position', p2 );
else
    f2 = [];
end

end
