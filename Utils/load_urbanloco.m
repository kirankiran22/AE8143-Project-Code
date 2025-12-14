function data = load_urbanloco(bag_path)
    % LOAD_URBANLOCO: Loads UrbanLoco ROS bag into KITTI-compatible struct.
    % Fixes: Uses '/navsat/fix' (Standard ROS) to avoid Header.Stamp errors.
    % Fixes: Uses '/imu_rt' for faster loading (matches GPS rate).
    
    fprintf('Loading ROS Bag: %s...\n', bag_path);
    try
        bag = rosbag(bag_path);
    catch
        error('ROS Toolbox not installed or file corrupted.');
    end
    
    % --- 1. Load Standard GPS (/navsat/fix) ---
    % Use this topic because it has the standard 'Header.Stamp' field
    topic_gps = '/navsat/fix'; 
    fprintf('  Extracting GPS (%s)...', topic_gps);
    
    b_gps = select(bag, 'Topic', topic_gps);
    msg_gps = readMessages(b_gps, 'DataFormat', 'struct');
    
    num_gps = length(msg_gps);
    if num_gps == 0
        error('No GPS messages found on topic %s', topic_gps);
    end
    
    gps_data = zeros(num_gps, 3);
    gps_time = zeros(num_gps, 1);
    
    % Set Start Time (t=0)
    t0 = double(msg_gps{1}.Header.Stamp.Sec) + double(msg_gps{1}.Header.Stamp.Nsec)*1e-9;
    
    for i = 1:num_gps
        gps_data(i, 1) = msg_gps{i}.Latitude;
        gps_data(i, 2) = msg_gps{i}.Longitude;
        gps_data(i, 3) = msg_gps{i}.Altitude;
        
        t = double(msg_gps{i}.Header.Stamp.Sec) + double(msg_gps{i}.Header.Stamp.Nsec)*1e-9;
        gps_time(i) = t - t0;
    end
    fprintf(' Done (%d points).\n', num_gps);
    
    % --- 2. Load IMU (/imu_rt) ---
    % Using 'rt' (Real Time) because it usually matches GPS rate (faster load)
    topic_imu = '/imu_rt'; 
    fprintf('  Extracting IMU (%s)...', topic_imu);
    
    b_imu = select(bag, 'Topic', topic_imu);
    msg_imu = readMessages(b_imu, 'DataFormat', 'struct');
    
    num_imu = length(msg_imu);
    imu_vals = zeros(num_imu, 6); 
    att_vals = zeros(num_imu, 3); 
    imu_time = zeros(num_imu, 1);
    
    fprintf(' Processing %d msgs: ', num_imu);
    
    for i = 1:num_imu
        % Linear Accel
        imu_vals(i, 1) = msg_imu{i}.LinearAcceleration.X;
        imu_vals(i, 2) = msg_imu{i}.LinearAcceleration.Y;
        imu_vals(i, 3) = msg_imu{i}.LinearAcceleration.Z;
        
        % Angular Rate
        imu_vals(i, 4) = msg_imu{i}.AngularVelocity.X;
        imu_vals(i, 5) = msg_imu{i}.AngularVelocity.Y;
        imu_vals(i, 6) = msg_imu{i}.AngularVelocity.Z;
        
        % Orientation (Quaternion -> Euler)
        q = [msg_imu{i}.Orientation.W, msg_imu{i}.Orientation.X, ...
             msg_imu{i}.Orientation.Y, msg_imu{i}.Orientation.Z];
        eul = quat2eul(q); % Returns ZYX [Yaw, Pitch, Roll]
        att_vals(i, :) = [eul(3), eul(2), eul(1)]; % Map to RPY
        
        t = double(msg_imu{i}.Header.Stamp.Sec) + double(msg_imu{i}.Header.Stamp.Nsec)*1e-9;
        imu_time(i) = t - t0;
        
        if mod(i, 1000) == 0, fprintf('.'); end
    end
    fprintf(' Done.\n');
    
    % --- 3. Synchronization ---
    fprintf('  Synchronizing Data...\n');
    
    % Remove duplicate IMU times if any
    [unique_imu_t, unique_idx] = unique(imu_time);
    imu_clean = imu_vals(unique_idx, :);
    att_clean = att_vals(unique_idx, :);
    
    % Interpolate to GPS time grid
    imu_synced = interp1(unique_imu_t, imu_clean, gps_time, 'linear', 'extrap');
    att_synced = interp1(unique_imu_t, att_clean, gps_time, 'linear', 'extrap');
    
    % --- 4. Pack Output ---
    data.num_frames = num_gps;
    data.timestamps = gps_time;
    data.gps = gps_data;
    data.imu = imu_synced;
    data.att = att_synced;
    
    % Store Objects for LiDAR loop
    data.bag_object = bag;
    data.lidar_selection = select(bag, 'Topic', '/rslidar_points');
    data.start_time_utc = datetime(t0, 'ConvertFrom', 'posixtime', 'TimeZone', 'UTC');
    data.drive_path = 'URBANLOCO_MODE'; 
    
    fprintf('UrbanLoco Load Complete.\n');
end