function mu_bw_seq = kalman_smoothing(mu_pred_fw_seq, sig_pred_fw_seq, ...
                                      mu_fw_seq, sig_fw_seq, ...
                                      U_t, rbt, sim_num_iter)

    mu_bw_seq = zeros(size(mu_fw_seq));
    mu_bw_seq(:,sim_num_iter+1) = mu_fw_seq(:,sim_num_iter+1);
    
    for idt = sim_num_iter : -1 : 1
        
        % Evaluate Jacobian and the precursor to Kalman smoother gain
        A = slam_JacF( mu_fw_seq(:,idt), U_t, rbt );
        Ks_pre = squeeze( sig_fw_seq(:,:,idt) ) * A';
        
        % Smoothed mean
        delta_mu_sf_1 = mu_bw_seq(:,idt+1) - mu_pred_fw_seq(:,idt+1);
        delta_mu = Ks_pre * linsolve( squeeze( sig_pred_fw_seq(:,:,idt+1 )), delta_mu_sf_1 );
        mu_bw_seq(:,idt) = mu_fw_seq(:,idt) + delta_mu;
    end

end
