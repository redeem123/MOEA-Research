%% UAV Path Planning Benchmarking Driver
% This script automates the execution of multi-objective algorithms across all problems.

clear; clc; format compact;

% Initialize paths
scriptDir = fileparts(mfilename('fullpath'));
run(fullfile(scriptDir, '..', 'startup.m'));

% Configuration
params.Generations = 500;
params.pop = 100;
params.Runs = 5;
% Metrics (HV/PD) are computed offline via analysis/compute_metrics.m.
params.computeMetrics = false;
% Parallel mode: 'parfeval' avoids parfor transparency errors on some setups.
params.useParallel = true;
params.parallelMode = 'parfeval';
params.resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');

% Get all benchmark problems
problemDir = fullfile(fileparts(mfilename('fullpath')), '..', 'problems');
problemFiles = dir(fullfile(problemDir, '*.mat'));

if ~isfolder(params.resultsDir)
    mkdir(params.resultsDir);
end

startTime = tic;
fprintf('Starting Benchmark Suite at %s\n', datestr(now));
fprintf('-------------------------------------------\n');

for i = 1:numel(problemFiles)
    % Load Problem
    fileName = problemFiles(i).name;
    load(fullfile(problemDir, fileName), 'terrainStruct');
    
    % Extract clean problem name
    problemName = strrep(fileName, 'terrainStruct_', '');
    problemName = strrep(problemName, '.mat', '');
    params.problemName = problemName;
    params.problemIndex = i;

    fprintf('Problem (%d/%d): %s\n', i, numel(problemFiles), problemName);      
    problemStart = tic;
    
    %--- Execute NMOPSO ---
    fprintf('>>> Running NMOPSO...\n');
    params.resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'results', 'NMOPSO');
    if ~isfolder(params.resultsDir), mkdir(params.resultsDir); end
    run_nmopso(terrainStruct, params);

    % --- Execute NSGA-II ---
    fprintf('>>> Running NSGA-II...\n');
    params.resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'results', 'NSGA-II');
    if ~isfolder(params.resultsDir), mkdir(params.resultsDir); end
    run_nsga2(terrainStruct, params); 
    
    fprintf('  - Completed in %.2f seconds\n', toc(problemStart));
    fprintf('-------------------------------------------\n');
end

totalTime = toc(startTime);
fprintf('Benchmark Suite Finished. Total time: %.2f seconds.\n', totalTime);

