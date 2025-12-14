% Visualizing Satellite Occlusion with LiDAR
clear; clc; close all;
addpath('Utils');

%Configuration
% 1. Dataset Selection
DATASET_MODE = 'URBANLOCO'; 
bag_file = 'Data/UrbanLoco/CA-20190828184706_blur_align-002.bag';
lidar_topic = '/rslidar_points'; % UrbanLoco specific topic
frame_idx = 1000;

fprintf('--------------------------------------------------\n');
fprintf('Visualizing Occlusion for UrbanLoco Frame: %d\n', frame_idx);

%% 1. Load GPS & IMU Data
fprintf('1. Loading Navigation Data from ROS Bag...\n');
meas_data = load_urbanloco(bag_file);

MAX_LIDAR_FRAMES = 2439; 
if frame_idx > MAX_LIDAR_FRAMES
    warning('Frame %d exceeds LiDAR limit. Clamping to %d.', frame_idx, MAX_LIDAR_FRAMES);
    frame_idx = MAX_LIDAR_FRAMES;
end

%% 2. Setup Satellites (Time Machine)
fprintf('2. Setting up Satellite Scenario...\n');
% Use the 2019 TLE for UrbanLoco
tleFile = 'Data/TLE/gps_2019_active.tle'; 

% Calculate exact time of this frame
current_time = meas_data.start_time_utc + seconds(meas_data.timestamps(frame_idx));

sc = satelliteScenario(current_time, current_time + minutes(1), 60);
sats = satellite(sc, tleFile);
car = groundStation(sc, meas_data.gps(frame_idx,1), meas_data.gps(frame_idx,2), ...
    'Altitude', meas_data.gps(frame_idx,3));

%% 3. Calculate Satellite Vectors (In NED Frame)
[az, el, r] = aer(car, sats, current_time);

% Mask angle 10 degrees
visible_mask = el > 10; 
vis_sats = sats(visible_mask);
vis_az = az(visible_mask);
vis_el = el(visible_mask);

fprintf('   Visible Satellites (Geometric): %d\n', length(vis_sats));

%% 4. Load LiDAR Cloud (UrbanLoco Specific)
fprintf('4. Loading LiDAR Frame...\n');

% Construct path to the extracted .bin file
meas_data.drive_path = 'Data/UrbanLoco'; 
lidar_fname = sprintf('%010d.bin', frame_idx-1);
lidar_path = fullfile(meas_data.drive_path, 'velodyne_points', 'data', lidar_fname);

if ~isfile(lidar_path)
    error('LiDAR file not found: %s\nDid you run the extraction script?', lidar_path);
end

% Read Binary (x,y,z,intensity)
fid = fopen(lidar_path, 'rb');
raw_data = fread(fid, [4 inf], 'single');
fclose(fid);
pt_cloud_velo = raw_data(1:3,:)'; % Nx3 Matrix

% Downsample for visualization speed
pt_cloud_velo = pt_cloud_velo(1:5:end, :);

%% 5. Coordinate Transforms: Satellites (NED) -> LiDAR Frame
% A. Get Car Orientation (Roll, Pitch, Yaw)
r_val = meas_data.att(frame_idx, 1);
p_val = meas_data.att(frame_idx, 2);
y_val = meas_data.att(frame_idx, 3);

y_ned = (pi/2) - y_val; % Adjust based on sensor mounting if needed
p_ned = -p_val;
r_ned = r_val;

R_body_to_ned = eul2rotm([y_ned, p_ned, r_ned], 'ZYX'); 

% B. Transform Satellite Vectors
[sx, sy, sz] = sph2cart(deg2rad(vis_az), deg2rad(vis_el), 1); 
sat_vecs_ned = [sx, sy, sz]'; 

% NED -> Body
R_ned_to_body = R_body_to_ned'; 
sat_vecs_body = R_ned_to_body * sat_vecs_ned;

% Body -> LiDAR
calib.T_velo_to_imu = eye(4); 
T_imu_to_velo = inv(calib.T_velo_to_imu);
R_body_to_velo = T_imu_to_velo(1:3, 1:3);

sat_vecs_lidar = R_body_to_velo * sat_vecs_body;

%% 6. Occlusion Logic (Ray Tracing)
fprintf('6. Running Ray Tracing...\n');
occlusion_radius = 1.0; 

% Normalize
sat_vecs_lidar = sat_vecs_lidar ./ vecnorm(sat_vecs_lidar);

% Run Check
[blocked_flags, ~] = check_occlusions(pt_cloud_velo, sat_vecs_lidar, occlusion_radius);

num_blocked = sum(blocked_flags);
fprintf('   Occlusion Check: %d Blocked, %d Visible.\n', num_blocked, length(blocked_flags) - num_blocked);

%% 7. Final Visualization
figure('Color', 'k', 'Name', ['UrbanLoco Frame ' num2str(frame_idx)]);
pcshow(pt_cloud_velo, 'MarkerSize', 20); 
hold on;
xlabel('X (Forward)'); ylabel('Y (Left)'); zlabel('Z (Up)');
title(sprintf('UrbanLoco Frame %d: Red=Blocked (%d), Green=Visible (%d)', ...
    frame_idx, num_blocked, length(blocked_flags)-num_blocked));

% Draw Rays
for i = 1:size(sat_vecs_lidar, 2)
    % Draw rays 60m long
    ray_end = sat_vecs_lidar(:, i)' * 60; 
    
    if blocked_flags(i)
        col = 'r';      
        lw = 3;         
        txt = ' [BLOCKED]';
    else
        col = 'g';      
        lw = 1;
        txt = '';
    end
    
    % Plot Ray
    plot3([0, ray_end(1)], [0, ray_end(2)], [0, ray_end(3)], ...
        'Color', col, 'LineWidth', lw, 'DisplayName', vis_sats(i).Name);
    
    % Label
    text(ray_end(1), ray_end(2), ray_end(3), ...
        [vis_sats(i).Name, txt], 'Color', 'w', 'FontSize', 8, 'Interpreter', 'none');
end

% Set View: Behind car
view(-170, 15); 
axis([-30 50 -40 40 -10 40]); 
grid on;