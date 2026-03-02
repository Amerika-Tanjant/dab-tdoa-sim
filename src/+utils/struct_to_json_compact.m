function s = struct_to_json_compact(st)
    s = jsonencode(st);
    s = strrep(s, '"', '""');
end
