%% Final Visualization
figure('Color', 'w', 'Name', 'Final Trajectory Analysis (Debugged)');


if ~exist('DATASET_MODE', 'var'), DATASET_MODE = 'KITTI'; end
if ~exist('lidar_deltas', 'var'), error('No lidar_deltas found. Run Step 3 first.'); end

lat0 = filt_data.gps(1,1);
lon0 = filt_data.gps(1,2);
alt0 = filt_data.gps(1,3);
wgs84 = wgs84Ellipsoid;

% --- 2. Process Ground Truth (GPS) ---
[n_gt_raw, e_gt_raw, ~] = geodetic2ned(filt_data.gps(:,1), filt_data.gps(:,2), filt_data.gps(:,3), ...
                               lat0, lon0, alt0, wgs84);

% --- 3. Process EKF Estimate ---
[n_ekf_raw, e_ekf_raw, ~] = geodetic2ned(est_path(:,1), est_path(:,2), est_path(:,3), ...
                                 lat0, lon0, alt0, wgs84);

% --- 4. Reconstruct Raw LiDAR
num_steps = size(lidar_deltas, 1);
lidar_pos_ned = zeros(num_steps, 3);
current_pos = [0;0;0];

for k = 1:num_steps
    d_body = lidar_deltas(k, :)';
    
    if exist('loop_indices', 'var'), idx = loop_indices(k); else, idx = k; end
    
    if strcmp(DATASET_MODE, 'URBANLOCO')
        heading = (pi/2) - meas_data.att(idx, 3); 
    else
        heading = (pi/2) - meas_data.att(idx, 3); 
    end
    
    R_yaw = [cos(heading) -sin(heading) 0; sin(heading) cos(heading) 0; 0 0 1];
    step_ned = R_yaw * [d_body(1); -d_body(2); -d_body(3)];
    
    current_pos = current_pos + step_ned;
    lidar_pos_ned(k, :) = current_pos';
end

n_lidar_raw = lidar_pos_ned(:,1);
e_lidar_raw = lidar_pos_ned(:,2);

% This removes any remaining offset due to projection errors
n_gt = n_gt_raw - n_gt_raw(1);
e_gt = e_gt_raw - e_gt_raw(1);

n_ekf = n_ekf_raw - n_ekf_raw(1);
e_ekf = e_ekf_raw - e_ekf_raw(1);

n_lidar = n_lidar_raw - n_lidar_raw(1);
e_lidar = e_lidar_raw - e_lidar_raw(1);

% --- 6. Plotting ---
hold on;
h1 = plot(e_gt, n_gt, 'k-', 'LineWidth', 2.5);
h2 = plot(e_lidar, n_lidar, '-', 'Color', [0 0.5 0], 'LineWidth', 1.5);
h3 = plot(e_ekf, n_ekf, 'm-.', 'LineWidth', 2);

% --- 7. Formatting ---
legend([h1, h2, h3], {'Ground Truth (GPS)', 'Raw LiDAR (Drift)', 'LiDAR-EKF (Fusion)'}, ...
    'Location', 'best', 'FontSize', 11);
xlabel('East Position (meters)');
ylabel('North Position (meters)');
title(sprintf('Sensor Fusion Performance (%s)', DATASET_MODE), 'Interpreter', 'none');
axis equal; grid on; set(gca, 'FontSize', 12);