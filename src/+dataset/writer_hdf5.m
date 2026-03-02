function writer_hdf5(h5path, sim, result, meta)
    if exist(h5path,'file'); delete(h5path); end
    tx = sim.tx_pos;
    uav = sim.uav_traj;
    K = size(uav,1);
    Ntx = size(tx,1);

    function put(name, data)
        if isempty(data); return; end
        sz = size(data);
        h5create(h5path, name, sz, 'Datatype', class(data));
        h5write(h5path, name, data);
    end

    put('/inputs/tx_pos_enu', tx);
    if isfield(sim,'ref_pos') && ~isempty(sim.ref_pos); put('/inputs/ref_pos_enu', sim.ref_pos); end
    put('/inputs/uav_traj_enu', uav);

    put('/measurements/toa_uav_s', result.measurements.toa_uav.toa_meas_s);
    if ~isempty(result.measurements.toa_ref); put('/measurements/toa_ref_s', result.measurements.toa_ref.toa_meas_s); end
    if ~isempty(result.measurements.dt_s); put('/measurements/dt_s', result.measurements.dt_s); end
    if ~isempty(result.measurements.ddt_s)
        put('/measurements/ddt_s', result.measurements.ddt_s);
        put('/measurements/dd_dist_m', result.measurements.dd_dist_m);
    end

    put('/labels/pos_true_enu', result.labels.pos_true_enu);
    if ~isempty(result.labels.pos_est_enu); put('/labels/pos_est_enu', result.labels.pos_est_enu); end

    put('/aux/vr_mps', result.aux.vr_mps);
    put('/aux/fd_hz', result.aux.fd_hz);
    if ~isempty(result.aux.weights); put('/aux/weights', result.aux.weights); end

    h5writeatt(h5path, '/', 'scenario_id', meta.scenario_id);
    h5writeatt(h5path, '/', 'run_id', meta.run_id);
    h5writeatt(h5path, '/', 'timestamp', meta.timestamp);
    h5writeatt(h5path, '/', 'K', K);
    h5writeatt(h5path, '/', 'Ntx', Ntx);
end
