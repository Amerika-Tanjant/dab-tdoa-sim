function v = velocity_from_traj(xyz, t)
    K = size(xyz,1);
    v = zeros(K,3);
    if K < 2; return; end
    dt = diff(t(:));
    dx = diff(xyz,1,1);
    v(1:K-1,:) = dx ./ dt;
    v(K,:) = v(K-1,:);
end
