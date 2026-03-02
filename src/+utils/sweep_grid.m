function grid = sweep_grid(sweeps)
%SWEEP_GRID Create a list of structs for cartesian product of sweep fields.
% sweeps: struct where each field is scalar or vector/list
% Output: grid(1..R) struct array, each has same fields with one chosen value.

    fn = fieldnames(sweeps);

    % Convert each field to cell array of values
    vals = cell(numel(fn),1);
    for i = 1:numel(fn)
        v = sweeps.(fn{i});

        if iscell(v)
            vals{i} = v;
        elseif isnumeric(v) || islogical(v)
            if isscalar(v)
                vals{i} = num2cell(v);
            else
                vals{i} = num2cell(v);
            end
        else
            % string/char
            vals{i} = {v};
        end
    end

    sizes = cellfun(@numel, vals);
    total = prod(sizes);

    % IMPORTANT: Preallocate with the correct fields
    tmp = struct();
    for i = 1:numel(fn)
        tmp.(fn{i}) = vals{i}{1};
    end
    grid = repmat(tmp, total, 1);

    % Mixed-radix counter for cartesian product
    idx = ones(1,numel(fn));
    for r = 1:total
        for i = 1:numel(fn)
            grid(r).(fn{i}) = vals{i}{idx(i)};
        end

        % increment
        for j = numel(fn):-1:1
            idx(j) = idx(j) + 1;
            if idx(j) <= sizes(j)
                break;
            else
                idx(j) = 1;
            end
        end
    end
end