function data = load_kitti(base_dir, drive_date, drive_id)
    % Robust KITTI Loader
    % Now extracts IMU (Accel/Gyro) data for Kalman Filtering
    
    % 1. Smart Path Detection
    path_v1 = fullfile(base_dir, drive_date, [drive_date, '_drive_', drive_id, '_sync']);
    path_v2 = fullfile(base_dir, [drive_date, '_drive_', drive_id, '_sync']);
    
    if exist(path_v1, 'dir')
        drive_path = path_v1;
    elseif exist(path_v2, 'dir')
        drive_path = path_v2;
    else
        error('Could not find drive folder!\nChecked:\n  %s\n  %s', path_v1, path_v2);
    end
    
    fprintf('Found Drive at: %s\n', drive_path);
    
    oxts_path = fullfile(drive_path, 'oxts', 'data');
    time_path = fullfile(drive_path, 'oxts', 'timestamps.txt');
    
    % 2. Load Timestamps
    if ~isfile(time_path)
        error('Timestamps file missing: %s', time_path);
    end
    
    fid = fopen(time_path, 'r');
    raw_times = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    
    t_objs = datetime(raw_times{1}, 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSSSSSSSS');
    data.timestamps = seconds(t_objs - t_objs(1)); 
    num_frames = length(data.timestamps);
    
    % 3. Load OXTS (GPS + IMU) Data
    % Pre-allocate for speed
    gps  = zeros(num_frames, 3); % Lat, Lon, Alt
    att  = zeros(num_frames, 3); % Roll, Pitch, Yaw
    imu  = zeros(num_frames, 6); % AccX, AccY, AccZ, GyroX, GyroY, GyroZ
    
    fprintf('Loading %d frames from OXTS...\n', num_frames);
    
    for i = 1:num_frames
        fname = sprintf('%010d.txt', i-1);
        val = importdata(fullfile(oxts_path, fname));
        
        % KITTI Format:
        % 1-3: Lat, Lon, Alt
        % 4-6: Roll, Pitch, Yaw
        % 10-12: Accel X, Y, Z (Body Frame)
        % 16-18: Gyro X, Y, Z (Body Frame)
        
        gps(i,:)  = val(1:3);
        att(i,:)  = val(4:6);
        imu(i,:)  = [val(10:12), val(16:18)]; % Extract IMU columns
        
        if mod(i, 100) == 0; fprintf('.'); end
    end
    fprintf('\n');
    
    % Pack Output
    data.num_frames = num_frames;
    data.gps = gps;
    data.att = att;
    data.imu = imu; % <--- This is the field the Kalman Filter needs
    data.drive_path = drive_path;
    data.start_time_utc = t_objs(1); 
    
    fprintf('KITTI Load Complete (GPS + IMU).\n');
end