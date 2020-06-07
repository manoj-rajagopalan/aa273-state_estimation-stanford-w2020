% Script to declutter the main program by containing the batch plots.

figure % Robot states.
for idx = 1:num_states_rbt
    h1(idx) = subplot( num_states_rbt, 1, idx );
    if( num_states_rbt == idx )
        mult = 180/pi;
    else
        mult = 1;
    end
    plot( tvec, [Xt_actual(idx,:)', ...
                 Xt_fast(idx,:)']*mult );
    grid on;
    switch( idx )
        case 1
            ylabel( 'P_x' );
            title( 'Robot positions and orientations, and estimates.' )
            legend( 'Actual', 'FastSLAM', ...
                    'Location', 'South', 'Orientation', 'Horizontal' );
        case 2
            ylabel( 'P_y' );
        case 3
            ylabel( '\theta (deg)' );
    end
end
xlabel( 'time (normalized to 1)' );
linkaxes( h1, 'x' );

for idx = 1:num_obst
    figure;
    clear h1
    id0 = num_states_rbt + (idx-1)*2; % Offset in state vector for this obstacle.
    for idy = 1:2
        h1(idy) = subplot( 2, 1, idy );
        plot( tvec, [Xt_actual(id0+idy,:)', ...
                     Xt_ekf(id0+idy,:)', ...
                     Xt_ukf(id0+idy,:)', ...
                     Xt_fast(id0+idy,:)'] );
        grid on;
        switch( idy )
            case 1
                ylabel( 'P_x' );
                title( sprintf( 'Obstacle %d positions and estimates.', idx ) )
                legend( 'Actual', 'FastSLAM', ...
                        'Location', 'South', 'Orientation', 'Horizontal' );
            case 2
                ylabel( 'P_y' );
        end
    end
    xlabel( 'time (normalized to 1)' );
    linkaxes( h1, 'x' );
end
