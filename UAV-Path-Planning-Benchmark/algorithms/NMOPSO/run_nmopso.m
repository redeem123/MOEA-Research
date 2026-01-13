function [bestScores, gen_hv] = run_nmopso(model, params)
    Generations = params.Generations;
    pop = params.pop;
    M = 4; problemIndex = 3; 
    
    % Override model resolution
    model.n = 20;
    
    resultsPath = fullfile(params.resultsDir, params.problemName);
    if ~isfolder(resultsPath), mkdir(resultsPath); end
    
    nVar = model.n; VarSize = [1 nVar];
    path_diag = norm(model.start - model.end);
    VarMax.r = 3 * path_diag / nVar; VarMin.r = VarMax.r / 9;
    AngleRange = pi/4;
    VarMin.psi = -AngleRange; VarMax.psi = AngleRange;          
    VarMin.phi = -AngleRange; VarMax.phi = AngleRange;          
    alpha_vel = 0.5;
    VelMax.r=alpha_vel*(VarMax.r-VarMin.r); VelMin.r=-VelMax.r;                    
    VelMax.psi=alpha_vel*(VarMax.psi-VarMin.psi); VelMin.psi=-VelMax.psi;                    
    VelMax.phi=alpha_vel*(VarMax.phi-VarMin.phi); VelMin.phi=-VelMax.phi;   
    
    nRep = 50; wdamp = 0.98; c1 = 1.5; c2 = 1.5;   
    nGrid = 5; alpha_grid = 0.1; beta = 2; gamma = 2; mu = 0.5; delta = 20;           
    
    computeMetrics = true;
    metricInterval = 100;
    initMaxTries = 10;

    parfor run = 1:params.Runs
        fprintf('  - Starting NMOPSO Run %d/%d\n', run, params.Runs);
        w = 1; 
        empty_particle = struct('Position',[], 'Velocity',[], 'Cost',[], ...
                                'Best',struct('Position',[], 'Cost',[]), ...
                                'IsDominated',[], 'GridIndex',[], 'GridSubIndex',[]);
        particle = repmat(empty_particle, pop, 1);
        
        % Initialize GlobalBest for this run
        GlobalBest = struct('Position', [], 'Cost', Inf(M, 1));
        
        % Initialization
        isInit = false;
        initTries = 0;
        while ~isInit && initTries < initMaxTries
            initTries = initTries + 1;
            for i = 1:pop
                particle(i).Position.r = unifrnd(VarMin.r, VarMax.r, VarSize);
                particle(i).Position.psi = unifrnd(VarMin.psi, VarMax.psi, VarSize);
                particle(i).Position.phi = unifrnd(VarMin.phi, VarMax.phi, VarSize);
                
                particle(i).Velocity.r = zeros(VarSize);
                particle(i).Velocity.psi = zeros(VarSize);
                particle(i).Velocity.phi = zeros(VarSize);
                
                cart_sol = NMOPSO_Utils.SphericalToCart(particle(i).Position, model);
                particle(i).Cost = NMOPSO_CostFunction(cart_sol, model, VarMin.r);
                
                particle(i).Best.Position = particle(i).Position;
                particle(i).Best.Cost = particle(i).Cost;
                
                if NMOPSO_Utils.Dominates(particle(i).Best, GlobalBest)
                    GlobalBest = particle(i).Best;
                    isInit = true;
                end
            end
        end
        if ~isInit
            allCosts = horzcat(particle.Cost)';
            sumCosts = sum(allCosts, 2);
            sumCosts(~isfinite(sumCosts)) = inf;
            [~, bestIdx] = min(sumCosts);
            GlobalBest = particle(bestIdx).Best;
        end
        
        particle = NMOPSO_Utils.DetermineDomination(particle);
        rep = particle(~[particle.IsDominated]);
        
        Grid = NMOPSO_Utils.CreateGrid(rep, nGrid, alpha_grid);
        for i_grid = 1:numel(rep)
            rep(i_grid) = NMOPSO_Utils.FindGridIndex(rep(i_grid), Grid);
        end
        
        local_gen_hv = zeros(Generations, 2);
        
        % Optimization Loop
        for it = 1:Generations
            fprintf('    - Run %d: Iteration %d/%d\n', run, it, Generations);
            
            % Optimization: Calculate leader probabilities once per generation
            
            for i = 1:pop
                leader = NMOPSO_Utils.SelectLeader(rep, beta);
                
                particle(i).Velocity.r = w*particle(i).Velocity.r ...
                    + c1*rand(VarSize).*(particle(i).Best.Position.r - particle(i).Position.r) ...
                    + c2*rand(VarSize).*(leader.Position.r - particle(i).Position.r);
                particle(i).Velocity.r = max(VelMin.r, min(VelMax.r, particle(i).Velocity.r));
                particle(i).Position.r = particle(i).Position.r + particle(i).Velocity.r;
                
                out_r = (particle(i).Position.r < VarMin.r | particle(i).Position.r > VarMax.r);
                particle(i).Velocity.r(out_r) = -particle(i).Velocity.r(out_r);
                particle(i).Position.r = max(VarMin.r, min(VarMax.r, particle(i).Position.r));
                
                particle(i).Velocity.psi = w*particle(i).Velocity.psi ...
                    + c1*rand(VarSize).*(particle(i).Best.Position.psi - particle(i).Position.psi) ...
                    + c2*rand(VarSize).*(leader.Position.psi - particle(i).Position.psi);
                particle(i).Velocity.psi = max(VelMin.psi, min(VelMax.psi, particle(i).Velocity.psi));
                particle(i).Position.psi = particle(i).Position.psi + particle(i).Velocity.psi;
                
                out_psi = (particle(i).Position.psi < VarMin.psi | particle(i).Position.psi > VarMax.psi);
                particle(i).Velocity.psi(out_psi) = -particle(i).Velocity.psi(out_psi);
                particle(i).Position.psi = max(VarMin.psi, min(VarMax.psi, particle(i).Position.psi));
                
                particle(i).Velocity.phi = w*particle(i).Velocity.phi ...
                    + c1*rand(VarSize).*(particle(i).Best.Position.phi - particle(i).Position.phi) ...
                    + c2*rand(VarSize).*(leader.Position.phi - particle(i).Position.phi);
                particle(i).Velocity.phi = max(VelMin.phi, min(VelMax.phi, particle(i).Velocity.phi));
                particle(i).Position.phi = particle(i).Position.phi + particle(i).Velocity.phi;
                
                out_phi = (particle(i).Position.phi < VarMin.phi | particle(i).Position.phi > VarMax.phi);
                particle(i).Velocity.phi(out_phi) = -particle(i).Velocity.phi(out_phi);
                particle(i).Position.phi = max(VarMin.phi, min(VarMax.phi, particle(i).Position.phi));

                cart_sol = NMOPSO_Utils.SphericalToCart(particle(i).Position, model);
                particle(i).Cost = NMOPSO_CostFunction(cart_sol, model, VarMin.r);
                
                if rand < (1-(it-1)/(Generations-1))^(1/mu)
                    NewSol_Local = struct();
                    NewSol_Local.Position = NMOPSO_Utils.Mutate(particle(i), rep, delta, VarMax, VarMin);
                    cart_new = NMOPSO_Utils.SphericalToCart(NewSol_Local.Position, model);
                    NewSol_Local.Cost = NMOPSO_CostFunction(cart_new, model, VarMin.r);
                    if NMOPSO_Utils.Dominates(NewSol_Local, particle(i))
                        particle(i).Position = NewSol_Local.Position;
                        particle(i).Cost = NewSol_Local.Cost;
                    elseif ~NMOPSO_Utils.Dominates(particle(i), NewSol_Local)
                        if rand < 0.5
                            particle(i).Position = NewSol_Local.Position;
                            particle(i).Cost = NewSol_Local.Cost;
                        end
                    end
                end
                
                if NMOPSO_Utils.Dominates(particle(i), particle(i).Best)
                    particle(i).Best.Position = particle(i).Position;
                    particle(i).Best.Cost = particle(i).Cost;
                elseif ~NMOPSO_Utils.Dominates(particle(i).Best, particle(i))
                    if rand < 0.5
                        particle(i).Best.Position = particle(i).Position;
                        particle(i).Best.Cost = particle(i).Cost;
                    end
                end
            end
            
            particle = NMOPSO_Utils.DetermineDomination(particle);
            rep = [rep; particle(~[particle.IsDominated])];
            rep = NMOPSO_Utils.DetermineDomination(rep);
            rep = rep(~[rep.IsDominated]);
            
            Grid = NMOPSO_Utils.CreateGrid(rep, nGrid, alpha_grid);
            for i_g = 1:numel(rep)
                rep(i_g) = NMOPSO_Utils.FindGridIndex(rep(i_g), Grid);
            end
            
            while numel(rep) > nRep
                rep = NMOPSO_Utils.DeleteOneRepMember(rep, gamma);
            end
            
            if ~isempty(rep)
                PopObj = horzcat(rep.Cost)';
                if size(PopObj, 2) ~= M, if size(PopObj, 1) == M, PopObj = PopObj'; end, end
                
                if computeMetrics && (mod(it, metricInterval) == 0 || it == 1 || it == Generations)
                    local_gen_hv(it, :) = [calMetric(1, PopObj, problemIndex, M), calMetric(2, PopObj, problemIndex, M)];
                elseif it > 1
                    local_gen_hv(it, :) = local_gen_hv(it-1, :);
                end
            end
            w = w * wdamp;
        end
        
        run_dir = fullfile(resultsPath, sprintf('Run_%d', run));
        if ~isfolder(run_dir), mkdir(run_dir); end
        save_data(fullfile(run_dir, 'gen_hv.mat'), local_gen_hv);
        
        if ~isempty(rep)
            PopObj = horzcat(rep.Cost)';
            runScores(run, :) = mean(PopObj, 1);
            for i_p = 1:numel(rep)
                local_dt_sv = struct();
                cart = NMOPSO_Utils.SphericalToCart(rep(i_p).Position, model);
                x_full = [model.start(1), cart.x, model.end(1)]';
                y_full = [model.start(2), cart.y, model.end(2)]';
                start_z = model.start(3);
                end_z = model.end(3);
                if isfield(model, 'safeH') && ~isempty(model.safeH)
                    start_z = model.safeH;
                    end_z = model.safeH;
                end
                z_full = [start_z, cart.z, end_z]';
                z_abs_path = zeros(size(x_full));
                for k=1:length(x_full)
                    xi = max(1, min(model.xmax, round(x_full(k))));
                    yi = max(1, min(model.ymax, round(y_full(k))));
                    z_abs_path(k) = z_full(k) + model.H(yi, xi);
                end
                local_dt_sv.path = [x_full, y_full, z_abs_path];
                local_dt_sv.objs = rep(i_p).Cost';
                save_data(fullfile(run_dir, sprintf('bp_%d.mat', i_p)), local_dt_sv);
            end
        end
    end
    save(fullfile(resultsPath, 'final_hv.mat'), 'runScores');
end

function save_data(filename, data)
    if isstruct(data)
        dt_sv = data; save(filename, 'dt_sv');
    else
        gen_hv = data; save(filename, 'gen_hv');
    end
end
