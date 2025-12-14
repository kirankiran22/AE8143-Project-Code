%% AE8143 Project: LiDAR-Aided GNSS Navigation
% 1. Load Data ->
% 2. Ray Tracing ->
% 3. LiDAR Odometry ->
% 4. EKF Fusion ->
% 5. Analysis

clear; clc; close all;
addpath('Utils');
rng(42);

%STEP 1: CONFIGURATION & LOADING
fprintf('--------------------------------------------------\n');
fprintf('Step 1: Loading Sensor Data...\n');

DATASET_MODE = 'URBANLOCO';
if strcmp(DATASET_MODE, 'KITTI')
   base_path = 'Data/KITTI';
   date_str = '2011_09_26';
   drive_str = '0005';
   meas_data = load_kitti(base_path, date_str, drive_str);
   calib = load_calibration(base_path, date_str);
   % KITTI TLE (2011)
   tleFile = 'Data/TLE/gps_2011_active.tle';
elseif strcmp(DATASET_MODE, 'URBANLOCO')
   % 1. Load the Bag
   bag_file = 'Data/UrbanLoco/CA-20190828184706_blur_align-002.bag';
   meas_data = load_urbanloco(bag_file);
  
   % 2. Set the path to the folder containing 'velodyne_points for kitti datasets'
   meas_data.drive_path = 'Data/UrbanLoco';
 
   % 3. frame count to the LiDAR limit (2439)
   % If we don't do this, Step 3 (ICP) will error when it runs out of .bin files
   lidar_count = 2439;
   meas_data.num_frames = min(meas_data.num_frames, lidar_count);
  
   % 4. Calibration & TLE
   calib.T_velo_to_imu = eye(4);
   tleFile = 'Data/TLE/gps_2019_active.tle';
end
startTime = meas_data.start_time_utc;
fprintf('Loaded %d frames. Start Time: %s\n', meas_data.num_frames, datestr(startTime));
fprintf('--------------------------------------------------\n');
fprintf('STEP 1: LOADING DATA\n');

base_path = 'Data/KITTI';

%FOR DATASET-1
%date_str = '2011_09_26';
%drive_str = '0005';

%FOR DATASET-2
date_str = '2011_09_29';
drive_str = '0071';

% 1. Load KITTI Data (GPS + IMU)
meas_data = load_kitti(base_path, date_str, drive_str);
calib = load_calibration(base_path, date_str);
startTime = meas_data.start_time_utc;

fprintf('  Loaded %d frames.\n', meas_data.num_frames);

% 2. Setup Satellites
sc = satelliteScenario(startTime, startTime + hours(1), 1.0);
tleFile = 'Data/TLE/gps_2011_active.tle';
sats = satellite(sc, tleFile);
fprintf('  Constellation Loaded: %d Satellites.\n', length(sats));
{

STEP 2: OCCLUSION ANALYSIS
fprintf('STEP 2: RUNNING RAY TRACING (VISIBILITY CHECK)\n');
Config: Process every frame for accuracy
step_size = 1;
loop_indices = 1:step_size:meas_data.num_frames;
num_steps = length(loop_indices);
Storage
visibility_log = cell(num_steps, 1);
time_log = zeros(num_steps, 1);
idx = 1;
Tuning: Sensitivity of the block check
0.5 = Permissive (Lets more GPS through) -> Good for testing convergence
5.0 = Strict (Blocks more GPS) -> Good for "Urban Canyon" stress test
occlusion_radius = 1.0;
Pre-calc Transforms
T_imu_velo = inv(calib.T_velo_to_imu);
R_body_velo = T_imu_velo(1:3, 1:3);
for k = loop_indices
   current_time = startTime + seconds(meas_data.timestamps(k));
  
   A. Geometric Visibility (Where are the sats?)
   car = groundStation(sc, meas_data.gps(k,1), meas_data.gps(k,2), 'Altitude', meas_data.gps(k,3));
   [az, el, ~] = aer(car, sats, current_time);
  
   Mask 1: Horizon (Must be > 10 deg up)
   geom_mask = el > 10;
   candidate_indices = find(geom_mask);
  
   if isempty(candidate_indices)
       visibility_log{idx} = {};
       idx = idx + 1; continue;
   end
  
   B. Coordinate Transforms (NED -> Body -> LiDAR)
   [sx, sy, sz] = sph2cart(deg2rad(az(candidate_indices)), deg2rad(el(candidate_indices)), 1);
   sat_vecs_ned = [sx, sy, sz]';
  
   r = meas_data.att(k, 1); p = meas_data.att(k, 2); y = meas_data.att(k, 3);
  
   Use our "Fixed" Rotation logic (matches run_ekf)
   y_ned = (pi/2) - y;
   p_ned = -p;
  
   R_body_ned = eul2rotm([y_ned, p_ned, r], 'ZYX');
  
   Note: R_body_ned maps Body->NED. We need NED->Body (Transpose) for the Ray.
   sat_vecs_lidar = R_body_velo * R_body_ned' * sat_vecs_ned;
  
   C. Occlusion Check
   lidar_file = sprintf('%010d.bin', k-1);
   path = fullfile(meas_data.drive_path, 'velodyne_points', 'data', lidar_file);
  
   if ~isfile(path)
       idx = idx+1; continue;
   end
  
   fid = fopen(path, 'rb'); raw = fread(fid, [4 inf], 'single'); fclose(fid);
   pt_cloud = raw(1:3,:)';
   Downsample strictly for speed in this check
   pt_cloud = pt_cloud(1:5:end, :);
  
   [occluded_subset, ~] = check_occlusions(pt_cloud, sat_vecs_lidar, occlusion_radius);
  
   D. Final List
   is_blocked = true(size(geom_mask));
   is_blocked(candidate_indices) = occluded_subset;
   usable_mask = geom_mask & (~is_blocked);
  
   usable_sats = sats(usable_mask);
   visibility_log{idx} = {usable_sats.Name};
   time_log(idx) = meas_data.timestamps(k);
  
   idx = idx + 1;
end
% --- STEP 3: LIDAR ODOMETRY (ICP) ---
fprintf('STEP 3: RUNNING LIDAR ODOMETRY\n');
Prepare Data Subset
filt_data = meas_data;
filt_data.gps = meas_data.gps(loop_indices, :);
filt_data.imu = meas_data.imu(loop_indices, :);
filt_data.att = meas_data.att(loop_indices, :);
filt_data.timestamps = meas_data.timestamps(loop_indices);
Run ICP
lidar_deltas = get_lidar_odometry(meas_data, loop_indices);

% --- STEP 4: SENSOR FUSION (EKF) & PATH GENERATION ---
fprintf('STEP 4: RUNNING EKF FUSION\n');

fprintf('  > Running Fusion Mode (GPS On)...\n');
[est_path, uncertainty, gnss_log, ~] = run_ekf(filt_data, sats, visibility_log, sc, lidar_deltas);

fprintf('  > Integrating Raw ICP Steps (No Filter)...\n');
lidar_ned = zeros(num_steps, 3);
current_pos_ned = [0; 0; 0];

ref_lat = meas_data.gps(1,1); ref_lon = meas_data.gps(1,2); ref_alt = meas_data.gps(1,3);
wgs84 = wgs84Ellipsoid;
for k = 1:num_steps
   if k > size(lidar_deltas, 1), break; end
  
   Get the step (Forward, Left, Up) from ICP
   d_lidar = lidar_deltas(k, :)';
  
   Get Heading from IMU (Required to know which way "Forward" is)
   y_kitti = meas_data.att(loop_indices(k), 3);
   heading = (pi/2) - y_kitti;
  
   2D Rotation Matrix (Yaw only)
   R = [cos(heading) -sin(heading) 0;
        sin(heading)  cos(heading) 0;
        0             0            1];
   step_body = [d_lidar(1); -d_lidar(2); -d_lidar(3)];
   step_ned = R * step_body;
  
   Accumulate
   current_pos_ned = current_pos_ned + step_ned;
   lidar_ned(k, :) = current_pos_ned';
end
[l_lat, l_lon, l_alt] = ned2geodetic(lidar_ned(:,1), lidar_ned(:,2), lidar_ned(:,3), ...
   ref_lat, ref_lon, ref_alt, wgs84);
lidar_only_path = [l_lat, l_lon, l_alt];


