function s = json_read(path)
    txt = fileread(path);
    s = jsondecode(txt);
end
