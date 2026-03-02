function run_all()
    addpath(genpath(pwd));
    masterPath = fullfile(fileparts(pwd),'configs','master_config.json');
    master = utils.json_read(masterPath);

    scenDir = fullfile(fileparts(pwd),'configs','scenarios');
    files = dir(fullfile(scenDir,'*.json'));
    if isempty(files)
        error('No scenario config files found in %s', scenDir);
    end

    fprintf('Found %d scenario configs.\n', numel(files));
    for i = 1:numel(files)
        scenPath = fullfile(files(i).folder, files(i).name);
        fprintf('\n=== Running scenario config: %s ===\n', files(i).name);
        try
            run_one(master, scenPath);
        catch ME
            warning('Scenario failed: %s\n%s', files(i).name, ME.getReport());
        end
    end
    fprintf('\nAll scenarios processed.\n');
end
