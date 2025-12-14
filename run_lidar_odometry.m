%% Run LiDAR Odometry (ICP Scan Matching)
% Calculates trajectory purely from Point Clouds, ignoring GPS and IMU.

clear; clc; close all;
addpath('Utils');

%Configuration
base_path = 'Data/KITTI'; 
date_str = '2011_09_26';
drive_str = '0005'; 

% Settings
step_size = 1;       % Must be 1 for ICP to work
downsample_grid = 0.5; % Grid filter size (Meters). Larger = Faster but less accurate.

%1. Load Data
fprintf('Loading Drive Metadata...\n');
meas_data = load_kitti(base_path, date_str, drive_str);
num_frames = meas_data.num_frames;

%Initialize State
% Global Pose (Transformation Matrix): Starts at Identity (0,0,0)
T_global = eye(4); 

% Storage
lidar_path = zeros(num_frames, 3);
prev_cloud = [];

fprintf('Starting ICP Loop (%d frames). This may take a moment...\n', num_frames);

% The ICP Loop
for k = 1:step_size:num_frames
    
    % A. Load Cloud
    lidar_file = sprintf('%010d.bin', k-1);
    path = fullfile(meas_data.drive_path, 'velodyne_points', 'data', lidar_file);
    if ~isfile(path), continue; end
    
    fid = fopen(path, 'rb'); raw = fread(fid, [4 inf], 'single'); fclose(fid);
    pt_cloud_raw = raw(1:3,:)'; 
    
    % B. Pre-process
    % 1. Create pointCloud object
    ptCloud = pointCloud(pt_cloud_raw);
    
    % 2. Downsample
    ptCloud = pcdownsample(ptCloud, 'gridAverage', downsample_grid);
    
    % C. Perform ICP
    if isempty(prev_cloud)
        prev_cloud = ptCloud;
        continue;
    end
    
    % MATCHING: Find transform that moves Current -> Previous
    try
        [tform, movingReg, rmse] = pcregistericp(ptCloud, prev_cloud, ...
            'Metric', 'pointToPlane', 'MaxIterations', 20, 'Tolerance', [0.01, 0.05]);
        
        % D. Update Global Pose
        T_global = T_global * tform.T;
        
        % E. Store Position
        lidar_path(k, :) = T_global(4, 1:3);
        
        % Update Previous Cloud
        prev_cloud = ptCloud;
        
    catch
        warning('ICP Failed at frame %d', k);
        lidar_path(k, :) = lidar_path(k-1, :); % Hold position
    end
    
    if mod(k, 10) == 0
        fprintf('Processed Frame %d / %d\n', k, num_frames);
    end
end

%% 4. Visualization (Trajectory Comparison)
figure('Color', 'w', 'Name', 'LiDAR Odometry vs GPS');

% A. Align Starts (Zero out the start position for comparison)
lidar_path_centered = lidar_path - lidar_path(2, :); % Start from first valid ICP

% Convert GPS to local meters
lat0 = meas_data.gps(1,1); lon0 = meas_data.gps(1,2); alt0 = meas_data.gps(1,3);
[n_gps, e_gps, ~] = geodetic2ned(meas_data.gps(:,1), meas_data.gps(:,2), meas_data.gps(:,3), ...
    lat0, lon0, alt0, wgs84Ellipsoid);

% Simple Plot (X vs Y from LiDAR)
plot(lidar_path_centered(:,1), lidar_path_centered(:,2), 'g.-', 'LineWidth', 2, 'DisplayName', 'LiDAR Odometry (ICP)');
hold on;
plot(e_gps, n_gps, 'b--', 'LineWidth', 2, 'DisplayName', 'GPS Ground Truth');

legend; axis equal; grid on;
xlabel('Local X (meters)'); ylabel('Local Y (meters)');
title('Trajectory: Pure LiDAR ICP vs GPS');