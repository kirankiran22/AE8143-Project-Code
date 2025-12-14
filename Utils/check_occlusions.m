function [is_blocked, min_dist] = check_occlusions(pt_cloud, sat_vecs, radius)
    % CHECK_OCCLUSIONS: Ray Casting to find blocked satellites.
    % UPDATED: Includes "Self-Collision" filter to ignore the car roof.
    
    if nargin < 3, radius = 1.0; end
    
    num_sats = size(sat_vecs, 2);
    is_blocked = false(1, num_sats);
    min_dist = zeros(1, num_sats);
    
    % --- FILTERING (The Fix) ---
    x = pt_cloud(:,1); 
    y = pt_cloud(:,2);
    z = pt_cloud(:,3);
    
    % Calculate distance from sensor (0,0,0)
    dist_sq = x.^2 + y.^2 + z.^2;
    
    % CRITICAL: Ignore points < 3.0m (The Car) AND points < -1.5m (The Road)
    % 3.0m covers the hood, trunk, and roof of the KITTI van.
    valid_idx = (dist_sq > 3.0^2) & (z > -1.5);
    
    clean_cloud = pt_cloud(valid_idx, :);
    
    % --- Standard Ray Tracing ---
    for i = 1:num_sats
        u = sat_vecs(:, i)';
        
        % 1. Project points onto ray
        projections = clean_cloud * u'; 
        
        % 2. Ignore points behind the ray start
        forward_mask = projections > 0;
        
        if ~any(forward_mask)
            min_dist(i) = 999; continue; 
        end
        
        pts = clean_cloud(forward_mask, :);
        proj = projections(forward_mask);
        
        % 3. Perpendicular Distance
        dist_vecs = pts - (proj * u);
        dists = sqrt(sum(dist_vecs.^2, 2));
        
        % 4. Check Collision
        closest = min(dists);
        min_dist(i) = closest;
        
        if closest < radius
            is_blocked(i) = true;
        end
    end
end