classdef NMOPSO_Utils
    methods(Static)
        
        function b = Dominates(x,y)
            if isstruct(x)
                x = x.Cost;
            end
            if isstruct(y)
                y = y.Cost;
            end
            % Minimization: x dominates y if x is better in all and strictly better in one
            % We also ensure costs are finite to be considered valid for dominance
            b = all(x <= y) && any(x < y) && all(x < 1e9); 
        end

        function pop = DetermineDomination(pop)
            nPop = numel(pop);
            for i = 1:nPop
                pop(i).IsDominated = false;
            end
            % Compare every pair
            for i = 1:nPop
                for j = 1:nPop
                    if i == j, continue; end
                    if NMOPSO_Utils.Dominates(pop(j), pop(i))
                        pop(i).IsDominated = true;
                        break;
                    end
                end
            end
        end

        function Grid = CreateGrid(pop, nGrid, alpha)
            c = [pop.Cost]; 
            cmin = min(c, [], 2); 
            cmax = max(c, [], 2);
            dc = cmax-cmin;
            cmin = cmin-alpha*dc;
            cmax = cmax+alpha*dc;
            nObj = size(c, 1); 
            empty_grid.LB = [];
            empty_grid.UB = [];
            Grid = repmat(empty_grid, nObj, 1);
            for j = 1:nObj
                cj = linspace(cmin(j), cmax(j), nGrid+1); 
                Grid(j).LB = [-inf, cj];
                Grid(j).UB = [cj, +inf];
            end
        end

        function particle = FindGridIndex(particle, Grid)
            nObj = numel(particle.Cost);
            nGrid = numel(Grid(1).LB)-2;
            particle.GridSubIndex = zeros(1, nObj);
            idx = zeros(1, nObj);
            for j = 1:nObj
                matches = find(particle.Cost(j) < Grid(j).UB, 1, 'first');
                if isempty(matches)
                    idx(j) = nGrid;
                else
                    idx(j) = matches;
                end
            end
            particle.GridSubIndex = idx;
            particle.GridIndex = particle.GridSubIndex(1);
            for j = 2:nObj
                particle.GridIndex = (particle.GridIndex-1) * nGrid + particle.GridSubIndex(j);
            end
        end

        function rep = DeleteOneRepMember(rep, gamma)
            GI = [rep.GridIndex];
            OC = unique(GI);
            N = zeros(size(OC));
            for k = 1:numel(OC)
                N(k) = sum(GI == OC(k));
            end
            P = exp(gamma*N);
            P = P/sum(P);
            sci = NMOPSO_Utils.RouletteWheelSelection(P);
            sc = OC(sci);
            SCM = find(GI == sc);
            sm = SCM(randi([1 numel(SCM)]));
            rep(sm) = [];
        end

        function i = RouletteWheelSelection(P)
            r = rand;
            C = cumsum(P); 
            i = find(r <= C, 1, 'first');
        end

        function leader = SelectLeader(rep, beta)
            GI = [rep.GridIndex];
            OC = unique(GI);
            N = zeros(size(OC));
            for k = 1:numel(OC)
                N(k) = sum(GI == OC(k));
            end
            P = exp(-beta*N);
            P = P/sum(P);
            sci = NMOPSO_Utils.RouletteWheelSelection(P);
            sc = OC(sci);
            SCM = find(GI == sc);
            sm = SCM(randi([1 numel(SCM)]));
            leader = rep(sm);
        end
        
        function xnew = Mutate(x,pm,delta,VarMax,VarMin)
            nVar = numel(x.Position.r);
            beta = tanh(delta*length(pm)); 
            
            xnew.r = x.Position.r + randn(1,nVar).*x.Best.Position.r*beta;
            xnew.phi = x.Position.phi + randn(1,nVar).*x.Best.Position.phi*beta;
            xnew.psi = x.Position.psi + randn(1,nVar).*x.Best.Position.psi*beta;

            xnew.r = max(VarMax.r, xnew.r);
            xnew.r = min(VarMin.r, xnew.r);

            xnew.phi = max(VarMax.phi, xnew.phi);
            xnew.phi = min(VarMin.phi, xnew.phi);

            xnew.psi = max(VarMax.psi, xnew.psi);
            xnew.psi = min(VarMin.psi, xnew.psi);
        end
        
        function A = TransfomationMatrix(r, phi, psi)
            % Returns a 4x4 transformation matrix
            cp = cos(phi); sp = sin(phi);
            cs = cos(-psi); ss = sin(-psi);
            
            Rot_z = [ cp, -sp, 0, 0; sp,  cp, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
            Rot_y = [ cs, 0, ss, 0; 0, 1, 0, 0; -ss, 0, cs, 0; 0, 0, 0, 1];
            Trans_x = [1 0 0 r; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            A = Rot_z * Rot_y * Trans_x;
        end

        function position = SphericalToCart(solution, model)
            % Optimized Spherical to Cartesian conversion
            % solution has fields r, phi, psi (1 x n)
            
            n = length(solution.r);
            xs = model.start(1); ys = model.start(2); zs = model.start(3);
            xf = model.end(1); yf = model.end(2); zf = model.end(3);
            if isfield(model, 'safeH') && ~isempty(model.safeH)
                zs = model.safeH;
                zf = model.safeH;
            end
            
            % Base start position and initial orientation towards target
            dirVector = [xf - xs; yf - ys; zf - zs];
            phistart = atan2(dirVector(2), dirVector(1));
            psistart = atan2(dirVector(3), norm(dirVector(1:2)));
            
            % Global start frame
            startMat = [1, 0, 0, xs; 0, 1, 0, ys; 0, 0, 1, zs; 0, 0, 0, 1];
            initialDir = NMOPSO_Utils.TransfomationMatrix(0, phistart, psistart);
            currentPos = startMat * initialDir;
            
            x = zeros(1, n); y = zeros(1, n); z = zeros(1, n);
            
            % Recursively apply transformations
            for i = 1:n
                T = NMOPSO_Utils.TransfomationMatrix(solution.r(i), solution.phi(i), solution.psi(i));
                currentPos = currentPos * T;
                x(i) = currentPos(1,4);
                y(i) = currentPos(2,4);
                z(i) = currentPos(3,4);
            end
            
            % Constraints (Absolute)
            x = max(model.xmin, min(model.xmax, x));
            y = max(model.ymin, min(model.ymax, y));
            z = max(model.zmin, min(model.zmax, z));
            
            position.x = x; position.y = y; position.z = z;
        end
    end
end
