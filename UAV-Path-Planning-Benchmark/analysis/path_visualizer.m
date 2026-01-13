function path_visualizer(problemName, runNum)
    % path_visualizer: 3D Visualization of a specific UAV path result
    
    % Get project root
    scriptPath = fileparts(mfilename('fullpath'));
    projectRoot = fullfile(scriptPath, '..');

    % Load terrain
    terrainFile = fullfile(projectRoot, 'problems', sprintf('terrainStruct_%s.mat', problemName));
    if ~exist(terrainFile, 'file')
        error('Terrain file not found: %s', terrainFile);
    end
    data = load(terrainFile);

    % Locate results
    runDir = fullfile(projectRoot, 'results', problemName, sprintf('Run_%d', runNum));
    d = dir(fullfile(runDir, 'bp_*.mat'));
    if isempty(d)
        error('No results found in %s', runDir);
    end
    
    % Pick a random path from the Pareto front to visualize
    sid = randi([1, length(d)], 1); 
    temp = load(fullfile(runDir, d(sid).name));
    sol = temp.dt_sv.path;

    terrain = data.terrainStruct.H;
    X = data.terrainStruct.X;
    Y = data.terrainStruct.Y;

    fig = figure('Name', sprintf('3D Path: %s', problemName));
    surf(X, Y, terrain, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
    colormap terrain;
    xlabel('X'); ylabel('Y'); zlabel('Altitude');
    title(sprintf('3D Path: %s (Run %d)', strrep(problemName, '_', ' '), runNum));
    colorbar;
    hold on;
    
    % Plot UAV path
    plot3(sol(:,1), sol(:,2), sol(:,3), 'r-', 'LineWidth', 3);
    % Start/End points
    scatter3(sol(1,1), sol(1,2), sol(1,3), 100, 'g', 'filled');
    scatter3(sol(end,1), sol(end,2), sol(end,3), 100, 'm', 'filled');
    
    legend('Terrain', 'UAV Path', 'Start', 'End');
    view(45, 30);
    grid on;
    hold off;
    
    % Save to plots folder
    plotDir = fullfile(projectRoot, 'results', 'Plots', 'Paths');
    if ~isfolder(plotDir), mkdir(plotDir); end
    saveas(fig, fullfile(plotDir, sprintf('Path_%s_run%d.png', problemName, runNum)));
end
