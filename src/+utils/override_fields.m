function base = override_fields(base, overrides)
    if isempty(overrides); return; end
    f = fieldnames(overrides);
    for i = 1:numel(f); base.(f{i}) = overrides.(f{i}); end
end
