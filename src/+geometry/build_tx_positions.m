function tx_pos = build_tx_positions(master)
    layout = master.tx_layout_default;
    if ~strcmpi(layout.coord_type,'ENU')
        error('Only ENU tx layout is supported in this starter repo.');
    end
    xy = layout.tx_enu_m;
    alt = layout.tx_alt_m;
    tx_pos = [xy(:,1), xy(:,2), alt*ones(size(xy,1),1)];
end
