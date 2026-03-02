function val = pick_sweep_value(default_list, params, key1, key2, fallback)
    if nargin < 5; fallback = []; end
    if isfield(params, key1); val = params.(key1); return; end
    if isfield(params, key2); val = params.(key2); return; end
    if ~isempty(default_list); val = default_list(1); return; end
    val = fallback;
end
