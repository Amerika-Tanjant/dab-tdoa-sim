function [E,N,U] = llh2enu(lat, lon, h, lat0, lon0, h0)
    m_per_deg_lat = 111320;
    lat0_rad = deg2rad(lat0);
    m_per_deg_lon = 111320 * cos(lat0_rad);
    dlat = lat - lat0;
    dlon = lon - lon0;
    N = dlat * m_per_deg_lat;
    E = dlon * m_per_deg_lon;
    U = h - h0;
end
