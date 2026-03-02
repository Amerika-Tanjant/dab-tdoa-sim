function llh = enu2llh(E,N,U, lat0, lon0, h0)
    m_per_deg_lat = 111320;
    lat0_rad = deg2rad(lat0);
    m_per_deg_lon = 111320 * cos(lat0_rad);
    lat = lat0 + (N / m_per_deg_lat);
    lon = lon0 + (E / m_per_deg_lon);
    h = h0 + U;
    llh = [lat, lon, h];
end
