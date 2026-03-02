function xhat = wls_position_solver(x0, ref_xy, tx_xy, z, w)
    if nargin < 5 || isempty(w); w = ones(1, numel(z)); end
    maxIter = 50;
    tol = 1e-6;
    x = x0(:);
    s1 = tx_xy(1,:).';
    for it = 1:maxIter
        m = size(tx_xy,1)-1;
        r = zeros(m,1);
        J = zeros(m,2);
        d1 = norm(x - s1);
        for k = 2:size(tx_xy,1)
            sk = tx_xy(k,:).';
            dk = norm(x - sk);
            fk = dk - d1;
            idx = k-1;
            r(idx) = fk - z(idx);
            if dk < 1e-9; gk = [0;0]; else; gk = (x - sk) / dk; end
            if d1 < 1e-9; g1 = [0;0]; else; g1 = (x - s1) / d1; end
            J(idx,:) = (gk - g1).';
        end
        W = diag(w(:));
        H = (J.'*W*J);
        g = (J.'*W*r);
        if rcond(H) < 1e-12; H = H + 1e-6*eye(2); end
        dx = -H \ g;
        if norm(dx) < tol; break; end
        alpha = 1.0;
        cost0 = r.'*W*r;
        while alpha > 1e-3
            x_try = x + alpha*dx;
            r_try = solvers.residuals_f(x_try, tx_xy, z);
            cost_try = r_try.'*W*r_try;
            if cost_try < cost0
                x = x_try; break;
            end
            alpha = alpha/2;
        end
        if alpha <= 1e-3
            x = x + dx;
        end
    end
    xhat = x(:).';
end
