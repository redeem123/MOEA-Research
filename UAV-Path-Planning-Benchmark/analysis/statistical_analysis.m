%% Research Analysis: Statistical Summary
% This script aggregates results across all problems and algorithms.

clear; clc;
% Initialize paths
scriptDir = fileparts(mfilename('fullpath'));
run(fullfile(scriptDir, '..', 'startup.m'));

resultsDir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');

% Identify Algorithm Folders
algoFolders = dir(resultsDir);
algoFolders = algoFolders([algoFolders.isdir]);
algoFolders = algoFolders(~strncmp({algoFolders.name}, '.', 1) & ~strcmp({algoFolders.name}, 'Plots'));

if isempty(algoFolders)
    error('No results found. Run scripts/run_benchmark.m first.');
end

for a = 1:numel(algoFolders)
    algoName = algoFolders(a).name;
    algoDir = fullfile(resultsDir, algoName);
    
    fprintf('\n======================================================\n');
    fprintf('Statistical Summary for Algorithm: %s\n', algoName);
    fprintf('======================================================\n');
    fprintf('%-30s | %-15s | %-15s\n', 'Problem Scenario', 'HV (Mean ± Std)', 'PD (Mean ± Std)');
    fprintf('%s\n', repmat('-', 1, 65));
    
    % Identify Problem Folders for this Algorithm
    probFolders = dir(algoDir);
    probFolders = probFolders([probFolders.isdir]);
    probFolders = probFolders(~strncmp({probFolders.name}, '.', 1));
    
    resultsTable = table();

    for i = 1:numel(probFolders)
        folderName = probFolders(i).name;
        hvFile = fullfile(algoDir, folderName, 'final_hv.mat');
        
        if exist(hvFile, 'file')
            load(hvFile, 'bestScores');
            
            cleanName = strrep(folderName, '_', '\_');
            
            % Metrics (bestScores: [Runs x M]) - Assuming HV is col 1, PD is col 2
            % Note: run_nsga2 and run_nmopso store mean objectives in runScores?
            % Wait, run_nsga2 saves:
            % runScores(run, :) = [calMetric(1...), calMetric(2...)];
            % So col 1 is HV, col 2 is PD.
            
            meanHV = mean(bestScores(:,1));
            stdHV  = std(bestScores(:,1));
            meanPD = mean(bestScores(:,2));
            stdPD  = std(bestScores(:,2));
            
            fprintf('%-30s | %6.4f ± %6.4f | %6.4f ± %6.4f\n', ...
                folderName, meanHV, stdHV, meanPD, stdPD);
            
            row = {cleanName, ...
                   sprintf('$%6.4f \pm %6.4f$', meanHV, stdHV), ...
                   sprintf('$%6.4f \pm %6.4f$', meanPD, stdPD)};
            resultsTable = [resultsTable; row];
        end
    end
    
    % Generate LaTeX Code
    fprintf('\n--- LaTeX Table Code for %s ---\n', algoName);
    fprintf('\begin{table}[ht]\n');
    fprintf('\centering\n');
    fprintf('\caption{Performance of %s on UAV Benchmark}\n', algoName);
    fprintf('\begin{tabular}{l|c|c}\n');
    fprintf('\hline\n');
    fprintf('Scenario & Hypervolume & Pure Diversity \\ \hline\n');
    for i = 1:size(resultsTable, 1)
        fprintf('%s & %s & %s \\\\ \n', ...
            resultsTable{i,1}{1}, resultsTable{i,2}{1}, resultsTable{i,3}{1});
    end
    fprintf('\hline\n');
    fprintf('\end{tabular}\n');
    fprintf('\end{table}\n');
end