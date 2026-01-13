function objs = evaluate_path(path, model)
    % evaluate_path: The unified objective evaluator for UAV Path Planning.
    % path: [N x 3] matrix of absolute coordinates (x, y, z)
    % model: terrainStruct containing H, nofly_c, nofly_r, nofly_h
    
    J_large = 1e9;
    
    % --- High-Resolution Interpolation for Collision Check ---
    % We interpolate the path to ensure we check every 0.5 units of distance
    % This prevents "skipping" over thin obstacles or gaps
    full_path = interpolate_path(path, 0.5); 
    
    % Basic path stats (using original path for length to assume efficient flight)
    % But for threat/collision, we use the detailed path
    
    % --- J1: Path Length ---
    diffs = diff(path, 1, 1);
    seg_lengths = sqrt(sum(diffs.^2, 2));
    J1 = sum(seg_lengths);
    
    % --- Collision & J2: Threat Violation ---
    J2 = 0;
    
    % 1. Ground Collision Check (High Res)
    collision = false;
    for i = 1:size(full_path, 1)
        x = full_path(i,1);
        y = full_path(i,2);
        z = full_path(i,3);
        
        % Boundary Check
        if x < model.xmin || x > model.xmax || y < model.ymin || y > model.ymax
            collision = true; break;
        end
        
        % Terrain Height Check
        xi = max(1, min(model.xmax, round(x)));
        yi = max(1, min(model.ymax, round(y)));
        groundH = model.H(yi, xi);
        
        if z < groundH
            collision = true; break;
        end
    end
    
    if collision
        % Return massive penalty for all objectives if crashed
        objs = [J_large, J_large, J_large, J_large];
        return;
    end
    
    % 2. No-Fly Zone Check (High Res)
    if isfield(model, 'nofly_c')
        center = model.nofly_c;
        radius = model.nofly_r;
        height = model.nofly_h;
        
        distToCenter = sqrt((full_path(:,1) - center(1)).^2 + (full_path(:,2) - center(2)).^2);
        inCircle = distToCenter <= radius;
        belowHeight = full_path(:,3) <= height;
        
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
    new_path = path(1,:);
    for i = 1:size(path,1)-1
        p1 = path(i,:);
        p2 = path(i+1,:);
        dist = norm(p2 - p1);
        num_steps = ceil(dist / step_size);
        if num_steps > 0
            % Generate points
            pts = zeros(num_steps, 3);
            for d = 1:3
                temp_pts = linspace(p1(d), p2(d), num_steps+1);
                pts(:,d) = temp_pts(2:end)';
            end
            new_path = [new_path; pts];
        end
    end
end