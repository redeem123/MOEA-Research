function objs = evaluate_path(path, model)
    % evaluate_path: The unified objective evaluator for UAV Path Planning.
    % path: [N x 3] matrix of absolute coordinates (x, y, z)
    % model: terrainStruct containing H, threats/nofly, and bounds
    %
    % Objectives (aligned with NMOPSOinOnefile MyCost):
    % J1: Path length consistency vs straight line
    % J2: Threat/no-fly penalty
    % J3: Altitude deviation (relative to zmin/zmax)
    % J4: Smoothness (turning angle energy)

    J_large = 1e9;

    if isempty(path) || size(path, 2) < 3
        objs = [J_large, J_large, J_large, J_large];
        return;
    end

    N = size(path, 1);
    x_all = path(:, 1)';
    y_all = path(:, 2)';
    z_abs = path(:, 3)';

    % Ground height at each point (rounded indices)
    if isfield(model, 'H')
        ground = SampleTerrain(x_all, y_all, model);
        if SegmentCollisionWithTerrain(x_all, y_all, z_abs, model)
            objs = [J_large, J_large, J_large, J_large];
            return;
        end
    else
        ground = zeros(size(x_all));
    end
    z_rel = z_abs - ground;

    % --- J1: Path length consistency ---
    diffs = diff(path, 1, 1);
    seg_lengths = sqrt(sum(diffs.^2, 2));
    minSegLen = 0;
    if isfield(model, 'start') && isfield(model, 'end') && isfield(model, 'n') && model.n > 0
        path_diag = norm(model.start - model.end);
        minSegLen = path_diag / (3 * model.n);
    end
    if minSegLen > 0 && any(seg_lengths <= minSegLen)
        J1 = J_large;
    else
        Traj = sum(seg_lengths);
        PP = norm(model.end - model.start);
        if Traj <= 0
            J1 = J_large;
        else
            J1 = abs(1 - PP/Traj);
        end
    end

    % --- J2: Threat/no-fly penalty ---
    J2 = 0;
    threats = [];
    if isfield(model, 'threats')
        threats = model.threats;
    elseif isfield(model, 'nofly_c') && isfield(model, 'nofly_r')
        c = model.nofly_c;
        if numel(c) >= 2
            threats = [c(1), c(2), 0, model.nofly_r];
        end
    end

    if ~isempty(threats)
        drone_size = 1;
        danger_dist = 10 * drone_size;
        J2_sum = 0;
        n2 = 0;
        collision = false;
        for i = 1:size(threats, 1)
            threat_x = threats(i, 1);
            threat_y = threats(i, 2);
            threat_radius = threats(i, 4);
            for j = 1:N-1
                dist = DistP2S([threat_x, threat_y], [x_all(j), y_all(j)], [x_all(j+1), y_all(j+1)]);
                if dist > (threat_radius + drone_size + danger_dist)
                    threat_cost = 0;
                elseif dist < (threat_radius + drone_size)
                    threat_cost = J_large;
                    collision = true;
                else
                    threat_cost = 1 - (dist - drone_size - threat_radius) / danger_dist;
                end
                n2 = n2 + 1;
                J2_sum = J2_sum + threat_cost;
            end
        end
        if n2 > 0
            J2 = J2_sum / n2;
        end
        if collision
            J2 = J_large;
        end
    end

    % --- J3: Altitude deviation ---
    if N > 2
        z_rel_use = z_rel(2:end-1);
    else
        z_rel_use = z_rel;
    end
    if any(z_rel_use < 0)
        J3 = J_large;
    else
        zmax = model.zmax;
        zmin = model.zmin;
        denom = (zmax - zmin) / 2;
        if denom <= 0
            J3 = J_large;
        else
            J3 = mean(abs(z_rel_use - (zmax + zmin) / 2) / denom);
        end
    end

    % --- J4: Smoothness ---
    if N < 3
        J4 = 0;
    else
        J4_sum = 0;
        n4 = 0;
        for i = 1:N-2
            for j = i:-1:1
                segment1 = [x_all(j+1); y_all(j+1); z_abs(j+1)] - [x_all(j); y_all(j); z_abs(j)];
                if nnz(segment1) ~= 0
                    break;
                end
            end
            for j = i:N-2
                segment2 = [x_all(j+2); y_all(j+2); z_abs(j+2)] - [x_all(j+1); y_all(j+1); z_abs(j+1)];
                if nnz(segment2) ~= 0
                    break;
                end
            end
            heading_angle = atan2(norm(cross(segment1, segment2)), dot(segment1, segment2));
            heading_angle = abs(heading_angle) / pi;
            n4 = n4 + 1;
            J4_sum = J4_sum + abs(heading_angle);
        end
        if n4 > 0
            J4 = J4_sum / n4;
        else
            J4 = 0;
        end
    end

    objs = [J1, J2, J3, J4];
end

function dist = DistP2S(x, a, b)
    d_ab = norm(a - b);
    d_ax = norm(a - x);
    d_bx = norm(b - x);
    if d_ab ~= 0
        if dot(a - b, x - b) * dot(b - a, x - a) >= 0
            A = [b - a; x - a];
            dist = abs(det(A)) / d_ab;
        else
            dist = min(d_ax, d_bx);
        end
    else
        dist = d_ax;
    end
end

function ground = SampleTerrain(xs, ys, model)
    xi = round(xs);
    yi = round(ys);
    h_rows = size(model.H, 1);
    h_cols = size(model.H, 2);
    xi = max(1, min(h_cols, xi));
    yi = max(1, min(h_rows, yi));
    idx = sub2ind([h_rows, h_cols], yi, xi);
    ground = model.H(idx);
end

function collision = SegmentCollisionWithTerrain(x_all, y_all, z_abs, model)
    collision = false;
    n = numel(x_all);
    for i = 1:n-1
        dx = x_all(i+1) - x_all(i);
        dy = y_all(i+1) - y_all(i);
        dz = z_abs(i+1) - z_abs(i);
        steps = max(2, ceil(max(abs(dx), abs(dy))) + 1);
        t = linspace(0, 1, steps);
        xs = x_all(i) + dx * t;
        ys = y_all(i) + dy * t;
        zs = z_abs(i) + dz * t;
        ground = SampleTerrain(xs, ys, model);
        if any(zs < ground)
            collision = true;
            return;
        end
    end
end
