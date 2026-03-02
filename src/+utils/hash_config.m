function run_id = hash_config(cfg)
    txt = jsonencode(cfg);
    md = java.security.MessageDigest.getInstance('MD5');
    md.update(uint8(txt));
    d = typecast(md.digest(),'uint8');
    hex = lower(dec2hex(d))';
    hex = hex(:)';
    run_id = hex(1:8);
end
