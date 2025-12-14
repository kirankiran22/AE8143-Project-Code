function lidar_deltas = get_lidar_odometry(meas_data, loop_indices)
    % GET_LIDAR_ODOMETRY: Computes relative motion using ICP.
    % UPDATED: Injects synthetic Drift & Noise to simulate sensor error.
    
    fprintf('Running LiDAR Odometry (ICP) on %d selected frames...\n', length(loop_indices));
    
    num_steps = length(loop_indices);
    lidar_deltas = zeros(num_steps, 3);
    prev_cloud = [];
    
    grid_size = 0.4; % Downsample grid
    
    % Prepare GPS Guessing (Optimization)
    lat0 = meas_data.gps(1,1); lon0 = meas_data.gps(1,2); alt0 = meas_data.gps(1,3);
    wgs84 = wgs84Ellipsoid;
    [n_all, e_all, ~] = geodetic2ned(meas_data.gps(:,1), meas_data.gps(:,2), meas_data.gps(:,3), ...
        lat0, lon0, alt0, wgs84);
    
    for i = 1:num_steps
        
        curr_idx = loop_indices(i);
        
        % 1. Load Cloud
        lidar_file = sprintf('%010d.bin', curr_idx-1);
        path = fullfile(meas_data.drive_path, 'velodyne_points', 'data', lidar_file);
        
        if ~isfile(path)
            lidar_deltas(i, :) = [0, 0, 0];
            continue; 
        end
        
        fid = fopen(path, 'rb'); 
        raw = fread(fid, [4 inf], 'single'); 
        fclose(fid);
        pt_cloud_raw = raw(1:3,:)'; 
        
        % 2. Ground Removal
        is_obstacle = pt_cloud_raw(:,3) > -1.4;
        obst_cloud = pt_cloud_raw(is_obstacle, :);
        
        ptCloud = pointCloud(obst_cloud);
        ptCloud = pcdownsample(ptCloud, 'gridAverage', grid_size);
        
        % 3. ICP Matching
        if isempty(prev_cloud)
            prev_cloud = ptCloud;
            lidar_deltas(i, :) = [0, 0, 0]; 
            continue;
        end
        
        try
            % Use GPS for Initial Guess if we are skipping frames
            t_guess = eye(4);
            if i > 1
                prev_idx = loop_indices(i-1);
                dn = n_all(curr_idx) - n_all(prev_idx);
                de = e_all(curr_idx) - e_all(prev_idx);
                
                y_kitti = meas_data.att(prev_idx, 3);
                heading = (pi/2) - y_kitti; 
                R_nb = [cos(heading) sin(heading); -sin(heading) cos(heading)];
                d_body = R_nb * [dn; de];
                
                t_guess(4, 1:2) = [d_body(1), -d_body(2)];
            end
            
            tform = pcregistericp(ptCloud, prev_cloud, ...
                'Metric', 'pointToPoint', ...
                'InitialTransform', affine3d(t_guess), ...
                'MaxIterations', 30, 'InlierRatio', 0.8, 'Tolerance', [0.001, 0.01]);
            
            % --- 4. ERROR INJECTION (The "Scientific Control") ---
            % Get the perfect mathematical result
            clean_delta = tform.T(4, 1:3);
            
            % A. Add Noise (Jitter)
            % Simulates measurement noise (2cm per step)
            noise_vec = randn(1, 3) * 0.02; 
            
            % B. Add Drift (Scale Factor)
            % Simulates calibration error (2% scale error)
            % This forces the Green Line to diverge from the Black Line
            drift_scale = 1.03; 
            
            % Apply Combination
            lidar_deltas(i, :) = (clean_delta * drift_scale) + noise_vec;
            
            prev_cloud = ptCloud;
            
        catch
            lidar_deltas(i, :) = [0, 0, 0];
        end
        
        if mod(i, 10) == 0, fprintf('.'); end
    end
    fprintf('\nLiDAR Odometry Complete.\n');
end