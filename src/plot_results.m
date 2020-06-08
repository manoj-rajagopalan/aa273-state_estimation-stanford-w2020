% Script to declutter the main program by containing the batch plots.
tvec = linspace( 0, sim_t_final, sim_num_iter+1 )';
if( any( any( ~isnan( Xt_pf(:,2:end) ) ) ) )
    legend_str1 = {'Actual', 'EKF', 'EKS', 'UKF', 'UKS', 'FastSLAM', 'PF'};
    legend_str2 = {'EKF', 'EKS', 'UKF', 'UKS', 'FastSLAM', 'Simulation', 'PF'};
else
    legend_str1 = {'Actual', 'EKF', 'EKS', 'UKF', 'UKS', 'FastSLAM'};
    legend_str2 = {'EKF', 'EKS', 'UKF', 'UKS', 'FastSLAM', 'Simulation'};
end

if( enable_plot_fast )
    % Plot the final state of FastSLAM
    plot_FinalState(Xt_actual, particles, map);
end


if( enable_plot_states )
    % Robot states.
    figure
    for idx = 1:num_states_rbt
        h1(idx) = subplot( num_states_rbt, 1, idx );
        if( num_states_rbt == idx )
            mult = 180/pi;
        else
            mult = 1;
        end
        plot( tvec, [Xt_actual(idx,:)', ...
                     Xt_ekf(idx,:)', ...
                     Xt_eks(idx,:)', ...
                     Xt_ukf(idx,:)', ...
                     Xt_uks(idx,:)', ...
                     Xt_fast(idx,:)', ...
                     Xt_pf(idx,:)']*mult );
        grid on;
        switch( idx )
            case 1
                ylabel( 'P_x' );
                title( 'Robot positions and orientations, and estimates.' )
                legend( legend_str1{:}, ...
                        'Location', 'South', 'Orientation', 'Horizontal' );
            case 2
                ylabel( 'P_y' );
            case 3
                ylabel( '\theta (deg)' );
        end
    end
    xlabel( 'time (normalized to 1)' );
    linkaxes( h1, 'x' );

    % Feature positions.
    for idx = 1:(num_states_obst/2)
        figure;
        clear h1
        id0 = num_states_rbt + (idx-1)*2; % Offset in state vector for this feature.
        for idy = 1:2
            h1(idy) = subplot( 2, 1, idy );
            plot( tvec, [Xt_actual(id0+idy,:)', ...
                         Xt_ekf(id0+idy,:)', ...
                         Xt_eks(id0+idy,:)', ...
                         Xt_ukf(id0+idy,:)', ...
                         Xt_uks(id0+idy,:)', ...
                         Xt_fast(id0+idy,:)', ...
                         Xt_pf(id0+idy,:)'] );
            grid on;
            switch( idy )
                case 1
                    ylabel( 'P_x' );
                    title( sprintf( 'Feature %d positions and estimates.', idx ) )
                    legend( legend_str1{:}, ...
                            'Location', 'South', 'Orientation', 'Horizontal' );
                case 2
                    ylabel( 'P_y' );
            end
        end
        xlabel( 'time (normalized to 1)' );
        linkaxes( h1, 'x' );
    end
end


if( enable_plot_measures )
    % Relative feature-to-robot measurements.
    for idx = 1:(num_states_obst/2)
        figure;
        clear h1
        id0 = (idx-1)*2; % Offset in measurement vector for this feature.
        for idy = 1:2
            h1(idy) = subplot( 2, 1, idy );
            if( 1 == idy )
                mult = 180/pi;
            else
                mult = 1;
            end
            plot( tvec, [dYt_ekf(id0+idy,:)', ...
                         dYt_eks(id0+idy,:)', ...
                         dYt_ukf(id0+idy,:)', ...
                         dYt_uks(id0+idy,:)', ...
                         dYt_fast(id0+idy,:)', ...
                         dYt_pf(id0+idy,:)']*mult );
            grid on;
            switch( idy )
                case 1
                    ylabel( 'bearing (deg)' );
                    title( sprintf( 'Feature %d measurement error: actual minus estimated.', idx ) )
                    legend( legend_str1{2:end}, ...
                            'Location', 'South', 'Orientation', 'Horizontal' );
                case 2
                    ylabel( 'range (normalized)' );
            end
        end
        xlabel( 'time (normalized to 1)' );
        linkaxes( h1, 'x' );
    end
end


if( enable_plot_map_no_time )
    % Environment and robot without time and robot orientation.
    id0 = 0;
    figure;
    plot( Xt_ekf(id0+1,:)', Xt_ekf(id0+2,:)', 'g.-', ...
          Xt_eks(id0+1,:)', Xt_eks(id0+2,:)', 'r.-', ...
          Xt_ukf(id0+1,:)', Xt_ukf(id0+2,:)', 'c.-', ...
          Xt_uks(id0+1,:)', Xt_uks(id0+2,:)', 'm.-', ...
          Xt_fast(id0+1,:)', Xt_fast(id0+2,:)', 'k.-', ...
          Xt_actual(id0+1,:)', Xt_actual(id0+2,:)', 'b.-', ...
          Xt_pf(id0+1,:)', Xt_pf(id0+2,:)', 'y.-' );
    xlabel( 'P_x' );
    ylabel( 'P_y' );
    grid on
    title( 'Path of Robot.' )
    legend( legend_str2 )

    for idx = 1:(num_states_obst/2)
        id0 = num_states_rbt + (idx-1)*2; % Offset in state vector for this feature.
        figure;
        plot( Xt_ekf(id0+1,:)', Xt_ekf(id0+2,:)', 'g.-', ...
              Xt_eks(id0+1,:)', Xt_eks(id0+2,:)', 'r.-', ...
              Xt_ukf(id0+1,:)', Xt_ukf(id0+2,:)', 'c.-', ...
              Xt_uks(id0+1,:)', Xt_uks(id0+2,:)', 'm.-', ...
              Xt_fast(id0+1,:)', Xt_fast(id0+2,:)', 'k.-', ...
              Xt_actual(id0+1,:)', Xt_actual(id0+2,:)', 'b.-', ...
              Xt_pf(id0+1,:)', Xt_pf(id0+2,:)', 'y.-' );
        xlabel( 'P_x' );
        ylabel( 'P_y' );
        grid on
        title( sprintf( 'Path of feature %d.', idx ) )
        legend( legend_str2 )
    end
end


if( enable_plot_metrics )
    % Metrics.
    figure
    h1(1) = subplot(4,1,1);
    plot( tvec, [Ks_ekf', Ks_ukf'] );
    grid on;
    ylabel( 'sum( \sigma( Kgain ) )' )
    legend( 'EKF', 'UKF' )
    title( 'Singular values (\sigma_i) of Kalman gain and \Sigma_{t|t}' );

    h1(2) = subplot(4,1,2);
    semilogy( tvec, [Kp_ekf', Kp_ukf'] );
    grid on;
    ylabel( 'prod( \sigma( Kgain ) )' )

    h1(3) = subplot(4,1,3);
    plot( tvec, [Ss_ekf', Ss_ukf'] );
    grid on
    ylabel( 'sum( \sigma( \Sigma ) )' )

    h1(4) = subplot(4,1,4);
    semilogy( tvec, [Sp_ekf', Sp_ukf'] );
    grid on
    ylabel( 'det = prod( \sigma( \Sigma ) )' )

    xlabel( 'time (normalized to 1)' );
    linkaxes( h1, 'x' );
end
