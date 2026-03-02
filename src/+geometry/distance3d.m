function d = distance3d(p, q)
    d = sqrt(sum((p-q).^2,2));
end
