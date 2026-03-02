function result = simulate_run(sim, runCfg)
    m = runCfg.master;
    p = runCfg.params;

    c = m.phy.c_mps;
    fs = m.phy.fs_hz;

    tx_pos = sim.tx_pos;
    Ntx = size(tx_pos,1);

    if isfield(sim,'ref_pos') && ~isempty(sim.ref_pos)
        ref_pos = sim.ref_pos;
    else
        ref_pos = [];
    end

    uav_traj = sim.uav_traj;
    if size(uav_traj,1) == 1 && ~isfield(sim,'time_s')
        K = 1;
        time_s = 0;
    else
        K = size(uav_traj,1);
        if isfield(sim,'time_s'); time_s = sim.time_s; else; time_s = (0:K-1)'*sim.dt_s; end
    end

    imp = m.impairments_default;
    imp = utils.override_fields(imp, p);

    if isfield(imp,'rx_offset_diff_s')
        rx_offset_diff = imp.rx_offset_diff_s;
    else
        rx_offset_diff = 0.0;
    end

    if isempty(ref_pos)
        toa_uav = measurements.toa_from_geometry(uav_traj, tx_pos, c, fs, imp);
        toa_ref = [];
    else
        toa_ref = measurements.toa_from_geometry(repmat(ref_pos, K, 1), tx_pos, c, fs, imp);
        toa_uav = measurements.toa_from_geometry(uav_traj, tx_pos, c, fs, imp);
        toa_uav.toa_meas_s = toa_uav.toa_meas_s + rx_offset_diff;
        toa_uav.toa_true_s = toa_uav.toa_true_s + rx_offset_diff;
    end

    if ~isempty(ref_pos)
        dt = measurements.tdoa_receivercentric(toa_uav.toa_meas_s, toa_ref.toa_meas_s);
        dt_true = measurements.tdoa_receivercentric(toa_uav.toa_true_s, toa_ref.toa_true_s);
    else
        dt = [];
        dt_true = [];
    end

    if ~isempty(ref_pos) && Ntx >= 2
        ddt = measurements.double_difference(dt);
        ddt_true = measurements.double_difference(dt_true);
        dd_dist_m = c * ddt;
        dd_dist_true_m = c * ddt_true;
    else
        ddt = []; ddt_true = [];
        dd_dist_m = []; dd_dist_true_m = [];
    end

    fc_list = m.rf.fc_list_hz;
    if numel(fc_list) < Ntx
        fc_used = [fc_list(:); repmat(fc_list(end), Ntx-numel(fc_list), 1)];
    else
        fc_used = fc_list(1:Ntx);
    end

    if size(uav_traj,1) >= 2
        v_traj = motion.velocity_from_traj(uav_traj(:,1:3), time_s);
    else
        v_traj = zeros(K,3);
    end
    [vr, fd] = measurements.doppler_model(uav_traj, v_traj, tx_pos, fc_used, c);

    if ~isempty(ddt)
        sigma = imp.sigma_toa_s;
        w = ones(K, Ntx-1) ./ max((2*sigma).^2, eps);
    else
        w = [];
    end

    pos_est = [];
    if ~isempty(ref_pos) && Ntx >= 4
        pos_est = zeros(K,3);
        x0 = uav_traj(1,1:2) + [10, -10];
        for k = 1:K
            z = solvers.build_z_vector(uav_traj(k,1:2), ref_pos(1:2), tx_pos(:,1:2), dd_dist_m(k,:));
            xhat = solvers.wls_position_solver(x0, ref_pos(1:2), tx_pos(:,1:2), z, w(min(k,size(w,1)),:));
            pos_est(k,:) = [xhat(:).', uav_traj(k,3)];
            x0 = xhat(:).';
        end
    end

    result = struct();

    result.measurements = struct();
    result.measurements.toa_uav = toa_uav;
    result.measurements.toa_ref = toa_ref;
    result.measurements.dt_s = dt;
    result.measurements.ddt_s = ddt;
    result.measurements.dd_dist_m = dd_dist_m;

    result.labels = struct();
    result.labels.pos_true_enu = uav_traj;
    result.labels.pos_est_enu = pos_est;
    result.labels.ref_pos_enu = ref_pos;
    result.labels.tx_pos_enu = tx_pos;

    result.aux = struct();
    result.aux.v_traj_mps = v_traj;
    result.aux.vr_mps = vr;
    result.aux.fd_hz = fd;
    result.aux.weights = w;

    summ = struct();
    summ.Ntx = Ntx;
    summ.K = K;
    summ.rx_offset_diff_s = rx_offset_diff;
    summ.sigma_toa_s = imp.sigma_toa_s;
    summ.p_nlos = imp.p_nlos;
    if ~isempty(pos_est)
        err = sqrt(sum((pos_est(:,1:2)-uav_traj(:,1:2)).^2,2));
        summ.pos_err_mean_m = mean(err);
        summ.pos_err_p95_m = prctile(err,95);
        summ.pos_err_max_m = max(err);
    end
    if ~isempty(dd_dist_m)
        summ.dd_dist_rms_m = sqrt(mean(dd_dist_m(:).^2));
    end
    result.summary = summ;
end
