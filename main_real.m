%% AE8143 Project: LiDAR-Aided GNSS Navigation

clear; clc; close all;
addpath('Utils');
rng(42); 

% STEP 1: CONFIGURATION & LOADING
fprintf('Step 1: Loading Sensor Data (UrbanLoco)... \n');
DATASET_MODE = 'URBANLOCO'; 

% 1. Load Raw Data
bag_file = 'Data/UrbanLoco/CA-20190828184706_blur_align-002.bag';
raw_data = load_urbanloco(bag_file); 
raw_data.drive_path = 'Data/UrbanLoco'; 

% 2. Define LiDAR Limit
lidar_count = 2439; 

% 3. RESAMPLE GPS (20Hz) TO MATCH LIDAR (10Hz)
% ensures the Black Line (GPS) covers the full drive duration
fprintf('  Resampling GPS (20Hz) to match LiDAR (10Hz)...\n');
sync_indices = round(linspace(1, raw_data.num_frames, lidar_count));

meas_data = raw_data;
meas_data.num_frames = lidar_count; 
meas_data.gps = raw_data.gps(sync_indices, :);
meas_data.imu = raw_data.imu(sync_indices, :);
meas_data.att = raw_data.att(sync_indices, :);
meas_data.timestamps = raw_data.timestamps(sync_indices);

% 4. Calibration & TLE
calib.T_velo_to_imu = eye(4); 
tleFile = 'Data/TLE/gps_2019_active.tle'; 

startTime = meas_data.start_time_utc;
fprintf('Loaded %d Synced Frames. Start Time: %s\n', meas_data.num_frames, datestr(startTime));

% Setup Satellites
sc = satelliteScenario(startTime, startTime + hours(1), 1.0); 
sats = satellite(sc, tleFile); 


%STEP 2: OCCLUSION ANALYSIS 
fprintf('STEP 2: RUNNING RAY TRACING (VISIBILITY CHECK)\n');

step_size = 1; 
loop_indices = 1:step_size:meas_data.num_frames;
num_steps = length(loop_indices);

% Storage
visibility_log = cell(num_steps, 1);
idx = 1;
occlusion_radius = 1.0; 

% Pre-calc Transforms
T_imu_velo = inv(calib.T_velo_to_imu);
R_body_velo = T_imu_velo(1:3, 1:3);

for k = loop_indices
    current_time = startTime + seconds(meas_data.timestamps(k));
    
    % A. Geometric Visibility
    car = groundStation(sc, meas_data.gps(k,1), meas_data.gps(k,2), 'Altitude', meas_data.gps(k,3));
    [az, el, ~] = aer(car, sats, current_time);
    
    geom_mask = el > 10;
    candidate_indices = find(geom_mask);
    
    if isempty(candidate_indices)
        visibility_log{idx} = {}; 
        idx = idx + 1; continue; 
    end
    
    % B. Coordinate Transforms
    [sx, sy, sz] = sph2cart(deg2rad(az(candidate_indices)), deg2rad(el(candidate_indices)), 1);
    sat_vecs_ned = [sx, sy, sz]';
    
    r = meas_data.att(k, 1); p = meas_data.att(k, 2); y = meas_data.att(k, 3);
    
    % URBANLOCO ROTATION LOGIC
    y_ned = (pi/2) - y;
    p_ned = -p;
    
    R_body_ned = eul2rotm([y_ned, p_ned, r], 'ZYX');
    sat_vecs_lidar = R_body_velo * R_body_ned' * sat_vecs_ned;
    
    % C. Occlusion Check
    lidar_file = sprintf('%010d.bin', k-1);
    path = fullfile(meas_data.drive_path, 'velodyne_points', 'data', lidar_file);
    
    if ~isfile(path)
        idx = idx+1; continue; 
    end
    
    fid = fopen(path, 'rb'); raw = fread(fid, [4 inf], 'single'); fclose(fid);
    pt_cloud = raw(1:3,:)'; 
    pt_cloud = pt_cloud(1:5:end, :); % Downsample
    
    [occluded_subset, ~] = check_occlusions(pt_cloud, sat_vecs_lidar, occlusion_radius);
    
    % D. Final List
    is_blocked = true(size(geom_mask));
    is_blocked(candidate_indices) = occluded_subset;
    usable_mask = geom_mask & (~is_blocked);
    
    usable_sats = sats(usable_mask);
    visibility_log{idx} = {usable_sats.Name};
    
    idx = idx + 1;
end

%STEP 3: LIDAR ODOMETRY (ICP)
fprintf('STEP 3: RUNNING LIDAR ODOMETRY\n');

filt_data = meas_data;
% Note: meas_data is already synced, pass it directly
lidar_deltas = get_lidar_odometry(meas_data, loop_indices);


%STEP 4: SENSOR FUSION (EKF)
fprintf('STEP 4: RUNNING EKF FUSION\n');
fprintf('  > Running Fusion Mode (GPS On)...\n');
[est_path, uncertainty, gnss_log, ~] = run_ekf(filt_data, sats, visibility_log, sc, lidar_deltas);


% STEP 5: FINAL ANALYSIS & DRIFT MEASUREMENT
fprintf('STEP 5: VISUALIZATION & ERROR METRICS (URBANLOCO)\n');

figure('Color', 'w', 'Name', 'Final Trajectory Analysis (UrbanLoco)');
hold on; grid on; axis equal;

% 1. Ground Truth (Aligned to 0,0)
lat0 = filt_data.gps(1,1);
lon0 = filt_data.gps(1,2);
alt0 = filt_data.gps(1,3);
wgs84 = wgs84Ellipsoid;

[n_gt_raw, e_gt_raw, ~] = geodetic2ned(filt_data.gps(:,1), filt_data.gps(:,2), filt_data.gps(:,3), ...
                                       lat0, lon0, alt0, wgs84);
n_gt = n_gt_raw - n_gt_raw(1);
e_gt = e_gt_raw - e_gt_raw(1);

% 2. EKF Path (Aligned to 0,0)
[n_ekf_raw, e_ekf_raw, ~] = geodetic2ned(est_path(:,1), est_path(:,2), est_path(:,3), ...
                                         lat0, lon0, alt0, wgs84);
n_ekf = n_ekf_raw - n_ekf_raw(1);
e_ekf = e_ekf_raw - e_ekf_raw(1);

% 3. Raw LiDAR Path (Reconstructed from Deltas)
lidar_pos_ned = zeros(num_steps, 3);
current_pos = [0;0;0];

for k = 1:num_steps
    d_body = lidar_deltas(k, :)';
    idx = loop_indices(k);
    
    heading = (pi/2) - meas_data.att(idx, 3); 
    
    R_yaw = [cos(heading) -sin(heading) 0; sin(heading) cos(heading) 0; 0 0 1];
    step_ned = R_yaw * [d_body(1); -d_body(2); -d_body(3)];
    
    current_pos = current_pos + step_ned;
    lidar_pos_ned(k, :) = current_pos';
end

n_lidar = lidar_pos_ned(:,1) - lidar_pos_ned(1,1);
e_lidar = lidar_pos_ned(:,2) - lidar_pos_ned(1,2);

% 4. Plotting
h1 = plot(e_gt, n_gt, 'k-', 'LineWidth', 2.5);
h2 = plot(e_lidar, n_lidar, '-', 'Color', [0 0.5 0], 'LineWidth', 1.5);
h3 = plot(e_ekf, n_ekf, 'm-.', 'LineWidth', 2);

legend([h1, h2, h3], {'Ground Truth (GPS)', 'Raw LiDAR (Drift)', 'LiDAR-EKF (Fusion)'}, ...
    'Location', 'best', 'FontSize', 11);
xlabel('East Position (m)'); ylabel('North Position (m)');
title('UrbanLoco: Trajectory Analysis');

% 5. Calculate & Print Error Stats
err_lidar = norm([n_gt(end), e_gt(end)] - [n_lidar(end), e_lidar(end)]);
err_ekf   = norm([n_gt(end), e_gt(end)] - [n_ekf(end), e_ekf(end)]);

fprintf('\n--- FINAL RESULTS (URBANLOCO) ---\n');
fprintf('Total Distance Traveled: %.2f m\n', sum(vecnorm(diff([n_gt, e_gt]), 2, 2)));
fprintf('Raw LiDAR Drift Error:   %.2f m\n', err_lidar);
fprintf('EKF Final Position Error: %.2f m\n', err_ekf);
fprintf('Improvement:             %.1f%%\n', ((err_lidar - err_ekf)/err_lidar)*100);
fprintf('--------------------------------------------------\n');