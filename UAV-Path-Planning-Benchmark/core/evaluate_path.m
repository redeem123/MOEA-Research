function objs = evaluate_path(path, model)
    % evaluate_path: The unified objective evaluator for UAV Path Planning.
    % path: [N x 3] matrix of absolute coordinates (x, y, z)
    % model: terrainStruct containing H, nofly_c, nofly_r, nofly_h
    %
    % Objectives:
    % J1: Path Length
    % J2: Threat Violation (No-fly zones)
    % J3: Average Altitude
    % J4: Smoothness (Bending Energy)

    J_large = 1e9;
    
    % Basic path stats
    diffs = diff(path, 1, 1);
    seg_lengths = sqrt(sum(diffs.^2, 2));
    
    % --- J1: Path Length ---
    J1 = sum(seg_lengths);
    
    % --- J2: Threat Violation ---
    J2 = 0;
    if isfield(model, 'nofly_c')
        center = model.nofly_c;
        radius = model.nofly_r;
        height = model.nofly_h;
        
        % Check segment midpoints for collision
        midpts = (path(1:end-1,:) + path(2:end,:)) / 2;
        distToCenter = sqrt((midpts(:,1) - center(1)).^2 + (midpts(:,2) - center(2)).^2);
        
        inCircle = distToCenter <= radius;
        belowHeight = midpts(:,3) <= height;
        
        if any(inCircle & belowHeight)
            % Heavy penalty + proportional cost
            J2 = J_large + sum(seg_lengths(inCircle & belowHeight));
        end
    end
    
    % --- J3: Altitude ---
    J3 = mean(path(:,3));
    
    % --- J4: Smoothness ---
    % Variations in Heading (horizontal) and Pitch (vertical)
    headings = atan2d(diffs(:,2), diffs(:,1));
    pitches = atand(diffs(:,3) ./ sqrt(diffs(:,1).^2 + diffs(:,2).^2));
    
    % Sum of squared changes
    J4 = sum(diff(headings).^2) + sum(diff(pitches).^2);
    
    objs = [J1, J2, J3, J4];
end
