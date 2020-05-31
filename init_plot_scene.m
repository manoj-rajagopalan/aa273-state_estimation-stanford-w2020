function [f1, f2] = init_plot_scene()

% Initialize figures.
f1 = figure;
f2 = figure;
p1 = get( f1, 'Position' );
p2 = get( f2, 'Position' );
p1(1) = 100;
p2(1) = p1(1) + p1(3) + 20;
set( f1, 'Position', p1 );
set( f2, 'Position', p2 );

end
