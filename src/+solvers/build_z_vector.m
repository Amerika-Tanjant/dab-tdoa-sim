function z = build_z_vector(uav_xy, ref_xy, tx_xy, dd_dist_m_row)
    Ntx = size(tx_xy,1);
    s1 = tx_xy(1,:);
    g = zeros(1,Ntx-1);
    for k = 2:Ntx
        sk = tx_xy(k,:);
        g(k-1) = norm(ref_xy - sk) - norm(ref_xy - s1);
    end
    z = dd_dist_m_row(:).' + g;
end
