function ddt = double_difference(dt)
    ddt = dt(:,2:end) - dt(:,1);
end
