function calib = load_calibration(base_dir, date_str)
% Reads KITTI calibration files to create transformation matrices.
% We specifically need:
%   1. T_velo_to_imu (To move LiDAR points to the GPS/Body frame)

calib_root = fullfile(base_dir, date_str);

% 1. Read IMU -> Velodyne (calib_imu_to_velo.txt)
% Note: KITTI gives T_imu_to_velo, but we usually want the inverse 
% (Velo -> IMU) to put the cloud in the body frame.
fid = fopen(fullfile(calib_root, 'calib_imu_to_velo.txt'), 'r');

% Parse line by line (Format: "R: xx xx xx...")
R_imu2velo = []; T_imu2velo = [];

while ~feof(fid)
    line = fgetl(fid);
    if startsWith(line, 'R:')
        % Read 9 float values
        vals = sscanf(line(3:end), '%f');
        R_imu2velo = reshape(vals, [3, 3])'; % Transpose because MATLAB fills columns first
    elseif startsWith(line, 'T:')
        % Read 3 float values
        vals = sscanf(line(3:end), '%f');
        T_imu2velo = vals;
    end
end
fclose(fid);

% Construct the 4x4 Homogeneous Matrix (IMU -> Velo)
T_imu_to_velo = eye(4);
T_imu_to_velo(1:3, 1:3) = R_imu2velo;
T_imu_to_velo(1:3, 4) = T_imu2velo;

% INVERT it to get Velo -> IMU (Body)
% This allows us to take a LiDAR point and assume it's attached to the GPS
calib.T_velo_to_imu = inv(T_imu_to_velo);

fprintf('Calibration Loaded: LiDAR -> IMU Transform ready.\n');
end