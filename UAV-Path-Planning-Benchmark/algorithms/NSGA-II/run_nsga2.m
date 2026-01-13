function [bestScores, gen_hv] = run_nsga2(model, params)
    % run_nsga2: Reusable NSGA-II implementation for UAV Path Planning
    % Inputs:
    %   model - terrainStruct with environment data
    %   params - struct with algorithm parameters (pop, Generations, Runs, resultsDir, problemName)
    
    Generations = params.Generations;
    pop = params.pop;
    M = 4; % Objectives: Length, Threat, Altitude, Smoothness
    problemIndex = 3; % Static for current benchmark
    
    % Boundary definition
    MinValue = [model.xmin, model.ymin, model.zmin];
    MaxValue = [model.xmax, model.ymax, model.zmax];
    Boundary = [MaxValue; MinValue];
    
    % Results storage
    runScores = zeros(params.Runs, M);
    resultsPath = fullfile(params.resultsDir, params.problemName);
    if ~isfolder(resultsPath)
        mkdir(resultsPath);
    end

    parfor run = 1:params.Runs
        fprintf('  - Starting Run %d/%d\n', run, params.Runs);
        
        % Local setup for parfor
        current_population = Chromosome.empty(0, pop);
        for i = 1 : pop
            p = Chromosome(model);
            p = initialize(p, model);
            p = evaluate(p, model);
            current_population(i) = p;
        end
        
        [~, FrontNo, CrowdDis] = EnvironmentalSelection(current_population, pop, pop, M);
        
        local_gen_hv = zeros(Generations, 2);
        for gen = 1:Generations
            if mod(gen, 100) == 0 || gen == 1
                fprintf('    - Run %d: Generation %d/%d\n', run, gen, Generations);
            end
            
            MatingPool = TournamentSelection(2, pop, FrontNo, -CrowdDis);
            offspring  = F_operator(current_population, MatingPool', Boundary, model);
            [current_population, FrontNo, CrowdDis] = EnvironmentalSelection([current_population, offspring], pop, pop*2, M);     
            
            obj = [current_population.objs];
            PopObj = reshape(obj, M, length(current_population))';
            
            if mod(gen, 50) == 0 || gen == 1 || gen == Generations
                local_gen_hv(gen, :) = [calMetric(1, PopObj, problemIndex, M), calMetric(2, PopObj, problemIndex, M)];
            elseif gen > 1
                local_gen_hv(gen, :) = local_gen_hv(gen-1, :);
            end
        end
        
        % Save run results
        run_dir = fullfile(resultsPath, sprintf('Run_%d', run));
        if ~isfolder(run_dir)
            mkdir(run_dir);
        end
        
        % Save individual run data
        save_data(fullfile(run_dir, 'gen_hv.mat'), local_gen_hv);
        
        % Final objectives
        obj = [current_population.objs];
        PopObj = reshape(obj, M, length(current_population))';
        runScores(run, :) = mean(PopObj, 1); % Store mean objective values for summary
        
        % Save best paths (Pareto front)
        for i = 1:size(current_population, 2)
            local_dt_sv = struct();
            local_dt_sv.path = current_population(i).path;
            local_dt_sv.objs = current_population(i).objs;
            save_data(fullfile(run_dir, sprintf('bp_%d.mat', i)), local_dt_sv);
        end
    end
    
    bestScores = runScores;
    save(fullfile(resultsPath, 'final_hv.mat'), 'bestScores');
end

function save_data(filename, data)
    % Helper to save data inside parfor
    if isstruct(data)
        dt_sv = data;
        save(filename, 'dt_sv');
    else
        gen_hv = data;
        save(filename, 'gen_hv');
    end
end
