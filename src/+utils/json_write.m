function json_write(path, s)
    txt = jsonencode(s, 'PrettyPrint', true);
    fid = fopen(path,'w');
    fwrite(fid, txt);
    fclose(fid);
end
