function sim = one_tx_two_rx_static_offset(runCfg)
    m = runCfg.master;
    s = runCfg.scenario;
    p = runCfg.params;

    tx_all = geometry.build_tx_positions(m);
    sim.tx_pos = tx_all(s.tx.use_tx_indices(1),:);

    D0  = utils.pick_sweep_value(s.ref.distances_m, p, 'ref_distances_m', 'ref_distance_m', 1500);
    th0 = utils.pick_sweep_value(s.ref.angles_deg, p, 'ref_angles_deg', 'ref_angle_deg', 0);
    h0  = utils.pick_sweep_value(s.ref.altitudes_m, p, 'ref_altitudes_m', 'ref_altitude_m', 10);

    D1  = utils.pick_sweep_value(s.uav.distances_m, p, 'uav_distances_m', 'uav_distance_m', 800);
    th1 = utils.pick_sweep_value(s.uav.angles_deg, p, 'uav_angles_deg', 'uav_angle_deg', 60);
    h1  = utils.pick_sweep_value(s.uav.altitudes_m, p, 'uav_altitudes_m', 'uav_altitude_m', 100);

    ref_enu = geometry.ring_point_enu(D0, th0);
    uav_enu = geometry.ring_point_enu(D1, th1);

    sim.ref_pos = [ref_enu(1), ref_enu(2), h0];
    sim.uav_traj = [uav_enu(1), uav_enu(2), h1];
    sim.dt_s = m.phy.frame_period_s;
end
