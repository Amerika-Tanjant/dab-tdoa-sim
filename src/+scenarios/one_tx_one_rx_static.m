function sim = one_tx_one_rx_static(runCfg)
    m = runCfg.master;
    s = runCfg.scenario;
    p = runCfg.params;

    tx_all = geometry.build_tx_positions(m);
    tx_idx = s.tx.use_tx_indices(1);
    sim.tx_pos = tx_all(tx_idx,:);

    D = utils.pick_sweep_value(s.rx.distances_m, p, 'distances_m', 'distance_m', 800);
    th = utils.pick_sweep_value(s.rx.angles_deg, p, 'angles_deg', 'angle_deg', 0);
    h  = utils.pick_sweep_value(s.rx.altitudes_m, p, 'altitudes_m', 'altitude_m', 50);

    rx_enu = geometry.ring_point_enu(D, th);
    sim.rx_pos = [rx_enu(1), rx_enu(2), h];
    sim.uav_traj = sim.rx_pos;
    sim.dt_s = m.phy.frame_period_s;
end
