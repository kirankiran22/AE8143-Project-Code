%% AE8143 Project: Full Drive Animation
clear; clc; close all;
addpath('Utils');

DATASET_MODE = 'URBANLOCO'; 

% Animation Settings
step_size = 5;          % Skip frames (1 = Realtime, 5 = Fast)
occlusion_radius = 1.0; % Sensitivity (meters)
save_video = true;      % Set true to save .avi file

fprintf('--------------------------------------------------\n');
fprintf('Initializing Animation for %s...\n', DATASET_MODE);

%% 1. Load Data based on Mode
if strcmp(DATASET_MODE, 'KITTI')
    base_path = 'Data/KITTI'; 
    date_str = '2011_09_26';
    drive_str = '0005';
    tleFile = 'Data/TLE/gps_2011_active.tle'; 
    
    meas_data = load_kitti(base_path, date_str, drive_str);
    calib = load_calibration(base_path, date_str);
    
    MAX_FRAMES = meas_data.num_frames;
    
elseif strcmp(DATASET_MODE, 'URBANLOCO')
    bag_file = 'Data/UrbanLoco/CA-20190828184706_blur_align-002.bag';
    tleFile = 'Data/TLE/gps_2019_active.tle'; 
    
    meas_data = load_urbanloco(bag_file);
    

    meas_data.drive_path = 'Data/UrbanLoco'; 
    calib.T_velo_to_imu = eye(4); 
    
    MAX_FRAMES = 2439; 
    meas_data.num_frames = min(meas_data.num_frames, MAX_FRAMES);
end

fprintf('Loaded %d frames.\n', meas_data.num_frames);

%% 2. Setup Satellite Scenario
fprintf('Initializing Orbit Propagator...\n');
drive_start = meas_data.start_time_utc + seconds(meas_data.timestamps(1));
drive_end   = meas_data.start_time_utc + seconds(meas_data.timestamps(end));

sc = satelliteScenario(drive_start, drive_end, 1.0); 
sats = satellite(sc, tleFile);

%% 3. Prepare Video Writer
if save_video
    v_name = sprintf('results_%s_video.avi', DATASET_MODE);
    v = VideoWriter(v_name);
    v.FrameRate = 10; % Playback speed
    open(v);
end

%% 4. Prepare Figure
fig = figure('Color', 'k', 'Position', [100, 100, 1000, 700]);
ax = axes('Parent', fig);
set(ax, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
view(-160, 20); % Initial Camera Angle

% Pre-calc Transform Constant (LiDAR -> Body)
T_imu_to_velo = inv(calib.T_velo_to_imu);
R_body_to_velo = T_imu_to_velo(1:3, 1:3);

%% 5. THE ANIMATION LOOP
loop_indices = 1:step_size:meas_data.num_frames;

for k = loop_indices
    
    % Time Handling
    current_time_sec = meas_data.timestamps(k);
    current_datetime = meas_data.start_time_utc + seconds(current_time_sec);
    
    % --- A. Update Car Position ---
    lat = meas_data.gps(k,1);
    lon = meas_data.gps(k,2);
    alt = meas_data.gps(k,3);
    
    car = groundStation(sc, lat, lon, 'Altitude', alt);
    
    % --- B. Get Satellite Angles ---
    [az, el, ~] = aer(car, sats, current_datetime);
    
    % Mask 5 deg (Horizon)
    vis_mask = el > 5; 
    vis_az = az(vis_mask);
    vis_el = el(vis_mask);
    current_sats = sats(vis_mask);
    
    % --- C. Load LiDAR ---
    % Construct path to binary file
    lidar_fname = sprintf('%010d.bin', k-1);
    lidar_path = fullfile(meas_data.drive_path, 'velodyne_points', 'data', lidar_fname);
    
    if ~isfile(lidar_path)
        continue;
    end
    
    fid = fopen(lidar_path, 'rb');
    raw = fread(fid, [4 inf], 'single');
    fclose(fid);
    pt_cloud = raw(1:3,:)'; 
    % pt_cloud = pt_cloud(1:5:end, :); % Optional: Downsample for speed
    
    % --- D. Coordinate Transforms ---
    % 1. Spherical -> NED Cartesian
    [sx, sy, sz] = sph2cart(deg2rad(vis_az), deg2rad(vis_el), 1);
    sat_vecs_ned = [sx, sy, sz]';
    
    % 2. NED -> Body (Handle Dataset Differences)
    r_val = meas_data.att(k, 1);
    p_val = meas_data.att(k, 2);
    y_val = meas_data.att(k, 3);
    
    if strcmp(DATASET_MODE, 'KITTI')
        % KITTI Standard
        R_body_to_ned = eul2rotm([y_val, p_val, r_val], 'ZYX');
    else
        % URBANLOCO Standard (Fix Yaw/Pitch)
        y_ned = (pi/2) - y_val;
        p_ned = -p_val;
        r_ned = r_val;
        R_body_to_ned = eul2rotm([y_ned, p_ned, r_ned], 'ZYX');
    end
    
    sat_vecs_body = R_body_to_ned' * sat_vecs_ned;
    
    % 3. Body -> LiDAR
    sat_vecs_lidar = R_body_to_velo * sat_vecs_body;
    
    % --- E. Occlusion Check ---
    [blocked, ~] = check_occlusions(pt_cloud, sat_vecs_lidar, occlusion_radius);
    
    % --- F. Draw Frame ---
    cla(ax);
    
    % Plot Cloud
    pcshow(pt_cloud, 'Parent', ax, 'MarkerSize', 20);
    colormap(ax, gray);
    hold(ax, 'on');
    
    % Plot Rays
    num_blocked = 0;
    for i = 1:length(current_sats)
        vec = sat_vecs_lidar(:,i)' * 50; % 50m Ray Length
        
        if blocked(i)
            c = 'r'; lw = 3; num_blocked = num_blocked + 1;
        else
            c = 'g'; lw = 1;
        end
        
        plot3(ax, [0, vec(1)], [0, vec(2)], [0, vec(3)], ...
            'Color', c, 'LineWidth', lw);
        
        % Label Blocked Satellites
        if blocked(i)
            text(ax, vec(1), vec(2), vec(3), current_sats(i).Name, 'Color', 'r');
        end
    end
    
    % View Config
    xlim(ax, [-30, 30]); ylim(ax, [-30, 30]); zlim(ax, [-5, 20]);
    title(ax, sprintf('%s Frame %d | Time: %.1fs | Blocked: %d/%d', ...
        DATASET_MODE, k, current_time_sec, num_blocked, length(current_sats)));
    
    drawnow limitrate;
    
    if save_video
        frame = getframe(fig);
        writeVideo(v, frame);
    end
    
    if mod(k, 20) == 0
        fprintf('Processed Frame %d... (%d Blocked)\n', k, num_blocked);
    end
end

if save_video
    close(v);
    fprintf('Video saved to %s\n', v_name);
end