function xy = traj_linear(start_enu, end_enu, t)
    T = max(t(end), eps);
    a = t(:) ./ T;
    xy = (1-a)*start_enu(:).' + a*end_enu(:).';
end
