function out = toa_from_geometry(rx_traj, tx_pos, c, fs, imp)
    K = size(rx_traj,1);
    Ntx = size(tx_pos,1);
    toa_true = zeros(K,Ntx);
    toa_meas = zeros(K,Ntx);

    for k = 1:K
        rx = rx_traj(k,:);
        d = sqrt(sum((tx_pos - rx).^2,2));
        tau = d ./ c;
        toa_true(k,:) = tau(:).';

        noise = imp.sigma_toa_s * randn(1,Ntx);

        bias = zeros(1,Ntx);
        if imp.p_nlos > 0
            flags = rand(1,Ntx) < imp.p_nlos;
            bias(flags) = imp.nlos_bias_mean_s + imp.nlos_bias_std_s*randn(1,sum(flags));
            bias(bias<0) = 0;
        end

        tau_meas = tau(:).' + noise + bias;

        if isfield(imp,'quantize_to_samples') && imp.quantize_to_samples
            samp = tau_meas * fs;
            samp_q = round(samp);
            tau_meas = samp_q / fs;
        end

        toa_meas(k,:) = tau_meas;
    end

    out = struct();
    out.toa_true_s = toa_true;
    out.toa_meas_s = toa_meas;
end
