function [vr, fd] = doppler_model(uav_pos, uav_vel, tx_pos, fc_hz, c)
    K = size(uav_pos,1);
    Ntx = size(tx_pos,1);
    vr = zeros(K,Ntx);
    fd = zeros(K,Ntx);
    for k = 1:K
        r = uav_pos(k,:);
        v = uav_vel(k,:);
        for n = 1:Ntx
            s = tx_pos(n,:);
            los = (s - r);
            dist = norm(los);
            if dist < eps; u = [0 0 0]; else; u = los / dist; end
            vr(k,n) = dot(v,u);
            fd(k,n) = (vr(k,n)/c) * fc_hz(n);
        end
    end
end
