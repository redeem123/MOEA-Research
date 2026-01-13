function objs = evaluate_path(path, model)
    % evaluate_path: The unified objective evaluator for UAV Path Planning.
    % path: [N x 3] matrix of absolute coordinates (x, y, z)
    % model: terrainStruct containing H, nofly_c, nofly_r, nofly_h
    
    J_large = 1e9;
    
    % --- High-Resolution Interpolation for Collision Check ---
    % Interpolate path to check collisions along segments.
    step_size = 1;
    if isfield(model, 'collisionStep') && isnumeric(model.collisionStep) && isfinite(model.collisionStep)
        step_size = model.collisionStep;
    end
    if step_size <= 0
        step_size = 1;
    end
    full_path = interpolate_path(path, step_size);
    
    % Basic path stats (using original path for length to assume efficient flight)
    % But for threat/collision, we use the detailed path
    
    % --- J1: Path Length ---
    diffs = diff(path, 1, 1);
    seg_lengths = sqrt(sum(diffs.^2, 2));
    J1 = sum(seg_lengths);
    
    % --- Collision & J2: Threat Violation ---
    J2 = 0;
    
    % 1. Ground Collision Check (High Res)
    x = full_path(:,1);
    y = full_path(:,2);
    z = full_path(:,3);

    % Boundary Check
    if any(x < model.xmin | x > model.xmax | y < model.ymin | y > model.ymax)
        objs = [J_large, J_large, J_large, J_large];
        return;
    end

    % Terrain Height Check
    xi = max(1, min(model.xmax, round(x)));
    yi = max(1, min(model.ymax, round(y)));
    groundH = model.H(sub2ind(size(model.H), yi, xi));

    if any(z < groundH)
        objs = [J_large, J_large, J_large, J_large];
        return;
    end
    
    % 2. No-Fly Zone Check (High Res)
    if isfield(model, 'nofly_c')
        center = model.nofly_c;
        radius = model.nofly_r;
        height = model.nofly_h;
        
        distToCenter = sqrt((x - center(1)).^2 + (y - center(2)).^2);
        inCircle = distToCenter <= radius;
        belowHeight = z <= height;
        
        violation_count = sum(inCircle & belowHeight);
        
        if violation_count > 0
            % Cost is proportional to samples inside
            J2 = J_large + violation_count;
        end
    end
    
    % --- J3: Altitude (Mean) ---
    J3 = mean(full_path(:,3));
    
    % --- J4: Smoothness (Original control points) ---
    % Smoothness is a property of the trajectory curve itself
    headings = atan2d(diffs(:,2), diffs(:,1));
    pitches = atand(diffs(:,3) ./ sqrt(diffs(:,1).^2 + diffs(:,2).^2));
    J4 = sum(diff(headings).^2) + sum(diff(pitches).^2);
    
    objs = [J1, J2, J3, J4];
end

function new_path = interpolate_path(path, step_size)
    % Linear interpolation between points to achieve desired resolution
    n_points = size(path, 1);
    n_segments = n_points - 1;
    steps_per_segment = zeros(n_segments, 1);
    for i = 1:n_segments
        dist = norm(path(i+1,:) - path(i,:));
        steps_per_segment(i) = max(1, ceil(dist / step_size));
    end

    total_points = 1 + sum(steps_per_segment);
    new_path = zeros(total_points, 3);
    new_path(1,:) = path(1,:);

    idx = 2;
    for i = 1:n_segments
        p1 = path(i,:);
        p2 = path(i+1,:);
        num_steps = steps_per_segment(i);
        t = (1:num_steps) / num_steps;
        new_path(idx:idx+num_steps-1, 1) = p1(1) + (p2(1) - p1(1)) * t;
        new_path(idx:idx+num_steps-1, 2) = p1(2) + (p2(2) - p1(2)) * t;
        new_path(idx:idx+num_steps-1, 3) = p1(3) + (p2(3) - p1(3)) * t;
        idx = idx + num_steps;
    end
end
