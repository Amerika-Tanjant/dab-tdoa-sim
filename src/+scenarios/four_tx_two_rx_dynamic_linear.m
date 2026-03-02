function sim = four_tx_two_rx_dynamic_linear(runCfg)
    m = runCfg.master;
    s = runCfg.scenario;
    p = runCfg.params;

    tx_all = geometry.build_tx_positions(m);
    sim.tx_pos = tx_all(s.tx.use_tx_indices,:);

    D0  = utils.pick_sweep_value(s.ref.distances_m, p, 'ref_distances_m', 'ref_distance_m', 1500);
    th0 = utils.pick_sweep_value(s.ref.angles_deg, p, 'ref_angles_deg', 'ref_angle_deg', 0);
    h0  = utils.pick_sweep_value(s.ref.altitudes_m, p, 'ref_altitudes_m', 'ref_altitude_m', 10);
    ref_enu = geometry.ring_point_enu(D0, th0);
    sim.ref_pos = [ref_enu(1), ref_enu(2), h0];

    traj = s.uav_traj;
    Ds  = utils.pick_sweep_value(traj.start_distances_m, p, 'start_distances_m', 'start_distance_m', 1500);
    De  = utils.pick_sweep_value(traj.end_distances_m, p, 'end_distances_m', 'end_distance_m', 2500);
    ths = utils.pick_sweep_value(traj.start_angles_deg, p, 'start_angles_deg', 'start_angle_deg', 30);
    the = utils.pick_sweep_value(traj.end_angles_deg, p, 'end_angles_deg', 'end_angle_deg', 210);
    v   = utils.pick_sweep_value(traj.speeds_mps, p, 'speeds_mps', 'speed_mps', 10);
    h   = utils.pick_sweep_value(traj.altitudes_m, p, 'altitudes_m', 'altitude_m', 100);

    start_enu = geometry.ring_point_enu(Ds, ths);
    end_enu   = geometry.ring_point_enu(De, the);

    if ischar(traj.dt_s) && strcmpi(traj.dt_s,'frame_period')
        dt = m.phy.frame_period_s;
    else
        dt = traj.dt_s;
    end

    if ischar(traj.duration_s) && strcmpi(traj.duration_s,'auto')
        dist = norm(end_enu - start_enu);
        T = max(dist / max(v, 0.1), dt);
    else
        T = traj.duration_s;
    end

    t = 0:dt:T;
    uav_xy = motion.traj_linear(start_enu, end_enu, t);
    uav_h  = motion.traj_altitude_constant(h, numel(t));

    sim.uav_traj = [uav_xy(:,1), uav_xy(:,2), uav_h(:)];
    sim.dt_s = dt;
    sim.time_s = t(:);
end
