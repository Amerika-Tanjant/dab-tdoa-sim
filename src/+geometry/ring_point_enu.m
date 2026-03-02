function enu = ring_point_enu(D_m, theta_deg)
    th = deg2rad(theta_deg);
    enu = [D_m*cos(th), D_m*sin(th)];
end
