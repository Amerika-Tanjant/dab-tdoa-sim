function r = residuals_f(x, tx_xy, z)
    s1 = tx_xy(1,:).';
    d1 = norm(x - s1);
    m = size(tx_xy,1)-1;
    r = zeros(m,1);
    for k = 2:size(tx_xy,1)
        sk = tx_xy(k,:).';
        dk = norm(x - sk);
        fk = dk - d1;
        r(k-1) = fk - z(k-1);
    end
end
