function run_one(master, scenarioConfigPath)
    addpath(genpath(pwd));
    scenario = utils.json_read(scenarioConfigPath);

    fc = master.rf.fc_list_hz;
    span = max(fc)-min(fc);
    if span > master.rf.usrp_bw_hz
        warning('RF span %.1f MHz exceeds USRP BW %.1f MHz.', span/1e6, master.rf.usrp_bw_hz/1e6);
    end

    outRoot = fullfile(fileparts(pwd), '..', master.io.output_root, scenario.id);
    if ~exist(outRoot, 'dir'); mkdir(outRoot); end

    idxPath = fullfile(outRoot, 'index.csv');
    if ~exist(idxPath,'file')
        fid = fopen(idxPath,'w');
        fprintf(fid, 'timestamp,scenario_id,run_id,output_dir,seed,summary_json\n');
        fclose(fid);
    end

    sweepGrid = utils.sweep_grid(scenario.sweeps);
    fprintf('Total runs to generate: %d\n', numel(sweepGrid));

    gen = scenario_factory(scenario.scenario_type);

    for r = 1:numel(sweepGrid)
        runParams = sweepGrid(r);
        runCfg.master = master;
        runCfg.scenario = scenario;
        runCfg.params = runParams;

        run_id = utils.hash_config(runCfg);

        outDir = fullfile(outRoot, ['run_' run_id]);
        if exist(outDir,'dir')
            fprintf('[%d/%d] run_%s already exists, skipping.\n', r, numel(sweepGrid), run_id);
            continue;
        end
        mkdir(outDir);

        if isfield(runParams,'seeds'); seed = runParams.seeds;
        elseif isfield(runParams,'seed'); seed = runParams.seed;
        else; seed = 1; end
        utils.set_seed(seed);

        fprintf('[%d/%d] Generating run_%s (seed=%d)\n', r, numel(sweepGrid), run_id, seed);

        sim = gen.generate(runCfg);
        result = simulate_run(sim, runCfg);

        meta = struct();
        meta.timestamp = datestr(datetime('now','TimeZone','UTC'), 'yyyy-mm-ddTHH:MM:SSZ');
        meta.scenario_id = scenario.id;
        meta.run_id = run_id;
        meta.seed = seed;
        meta.master = master;
        meta.scenario = scenario;
        meta.run_params = runParams;
        meta.sim_summary = result.summary;

        utils.json_write(fullfile(outDir, 'meta.json'), meta);
        if master.io.write_hdf5
            dataset.writer_hdf5(fullfile(outDir,'data.h5'), sim, result, meta);
        end

        fid = fopen(idxPath,'a');
        summary_json = utils.struct_to_json_compact(result.summary);
        fprintf(fid, '%s,%s,%s,%s,%d,"%s"\n', meta.timestamp, scenario.id, run_id, outDir, seed, summary_json);
        fclose(fid);
    end
end
