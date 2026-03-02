function ok = in_roi_radius(enu_xy, radius_m)
    ok = (sqrt(enu_xy(1).^2 + enu_xy(2).^2) <= radius_m);
end
