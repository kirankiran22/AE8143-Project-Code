function [est_lla, P_trace, gnss_log, dop_log] = run_ekf(meas_data, sats, visibility_log, sc, lidar_deltas)
    % RUN_EKF: Integrated GNSS/INS/LiDAR EKF
    % Tuned to correct LiDAR Drift.
    
    fprintf('Initializing Kalman Filter (GNSS + IMU + LiDAR + DOP)...\n');
    
    num_steps = length(visibility_log);
    
    %1. Calculate Initial dt
    if length(meas_data.timestamps) > 1
        dt = double(meas_data.timestamps(2) - meas_data.timestamps(1));
    else
        dt = 0.1; 
    end
    
    % 2. Coordinate Setup
    ref_lat = meas_data.gps(1,1); ref_lon = meas_data.gps(1,2); ref_alt = meas_data.gps(1,3);
    wgs84 = wgs84Ellipsoid;
    
    %3. State Initialization
    x = zeros(8, 1);
    [n2, e2, d2] = geodetic2ned(meas_data.gps(2,1), meas_data.gps(2,2), meas_data.gps(2,3), ...
                                ref_lat, ref_lon, ref_alt, wgs84);
    x(4:6) = [n2; e2; d2] / dt;
    
    % 4. Tuning
    P = eye(8) * 10; 
    
    % Process Noise (Q):
    % High Position Noise (20.0) -> Allows filter to jump/snap to GPS
    % High Velocity Noise (10.0) -> Allows sharp turns
    Q = diag([20.0, 20.0, 20.0,  10.0, 10.0, 10.0,  1, 1]); 
    
    % Measurement Noise (R):
    sigma_gps = 0.5^2;      
    
    % High LiDAR Noise
    sigma_lidar = 5.0^2;  
    
    % Storage
    est_ned = zeros(num_steps, 3);
    P_trace = zeros(num_steps, 1);
    gnss_log = []; 
    dop_log = zeros(num_steps, 4); 
    
    for k = 1:num_steps
        
        % A. DYNAMIC TIME STEP
        if k > 1
            t_curr = meas_data.timestamps(k);
            t_prev = meas_data.timestamps(k-1);
            dt = double(t_curr - t_prev);
            if dt > 1.0, dt = 0.1; end
        end
        
        %PREDICTION
        r_kitti = meas_data.att(k, 1);
        p_kitti = meas_data.att(k, 2);
        y_kitti = meas_data.att(k, 3);
        
        % Align Orientation
        y = (pi/2) - y_kitti; 
        r = r_kitti; p = -p_kitti; 
        
        % Align Acceleration
        raw_acc = meas_data.imu(k, 1:3)'; 
        acc_body = [raw_acc(1); -raw_acc(2); -raw_acc(3)];
        
        R_body_to_ned = eul2rotm([y, p, r], 'ZYX');
        acc_ned = R_body_to_ned * acc_body;
        
        % State Propagation
        Phi = eye(8);
        Phi(1,4) = dt; Phi(2,5) = dt; Phi(3,6) = dt;
        Phi(7,8) = dt;
        
        x = Phi * x;
        x(4:6) = x(4:6) + (acc_ned * dt);
        P = Phi * P * Phi' + Q;
        
        %UPDATE (LiDAR Odometry)
        if k <= size(lidar_deltas, 1) && any(lidar_deltas(k,:))
            d_lidar = lidar_deltas(k, :)'; 
            
            % Velocity (Body Frame FRD)
            vel_body = [d_lidar(1); -d_lidar(2); -d_lidar(3)] / dt;
            z_lidar_vel = R_body_to_ned * vel_body;
            
            H_lidar = zeros(3, 8);
            H_lidar(1:3, 4:6) = eye(3);
            
            % Update
            R_lidar = eye(3) * sigma_lidar;
            K = P * H_lidar' * inv(H_lidar * P * H_lidar' + R_lidar);
            x = x + K * (z_lidar_vel - H_lidar * x);
            P = (eye(8) - K * H_lidar) * P;
        end
        
        %UPDATE (GPS)
        valid_sat_names = visibility_log{k};
        dop_curr = [99.9, 99.9, 99.9, 99.9]; 
        
        if length(valid_sat_names) >= 4
            current_time = meas_data.start_time_utc + seconds(meas_data.timestamps(k));
            sat_indices = ismember({sats.Name}, valid_sat_names);
            active_sats = sats(sat_indices);
            
            if ~isempty(active_sats)
                [sat_pos_ecef, ~] = states(active_sats, current_time, 'CoordinateFrame', 'ecef');
                
                [el, elo, eh] = ned2geodetic(x(1), x(2), x(3), ref_lat, ref_lon, ref_alt, wgs84);
                [ex, ey, ez] = geodetic2ecef(el, elo, eh, wgs84);
                est_ecef = [ex, ey, ez];
                [tx, ty, tz] = geodetic2ecef(meas_data.gps(k,1), meas_data.gps(k,2), meas_data.gps(k,3), wgs84);
                
                num_meas = length(active_sats);
                z = zeros(num_meas, 1); h_x = zeros(num_meas, 1); 
                H = zeros(num_meas, 8); G = zeros(num_meas, 4);
                [~, ~, ~, R_en] = ecef2ned(ex, ey, ez, ref_lat, ref_lon, ref_alt, wgs84);

                for j = 1:num_meas
                    s_j = sat_pos_ecef(1:3, j)';
                    z(j) = norm(s_j - [tx, ty, tz]) + randn*sqrt(sigma_gps) + x(7);
                    
                    range_vec = s_j - est_ecef;
                    pred_r = norm(range_vec);
                    h_x(j) = pred_r + x(7);
                    
                    unit_ecef = range_vec / pred_r;
                    unit_ned = R_en * unit_ecef';
                    
                    H(j, 1:3) = -unit_ned'; H(j, 7) = 1;
                    G(j, 1:3) = unit_ned';  G(j, 4) = 1;
                    
                    gnss_log = [gnss_log; k, j, z(j), true_dist, z(j)-h_x(j)];
                end

                try
                    Q_dop = inv(G' * G);
                    gdop = sqrt(trace(Q_dop));
                    pdop = sqrt(Q_dop(1,1)+Q_dop(2,2)+Q_dop(3,3));
                    hdop = sqrt(Q_dop(1,1)+Q_dop(2,2));
                    vdop = sqrt(Q_dop(3,3));
                    dop_curr = [gdop, pdop, hdop, vdop];
                catch
                end
                
                % Update
                R_mat = eye(num_meas) * sigma_gps;
                K = P * H' * inv(H * P * H' + R_mat);
                dx = K * (z - h_x);
                x = x + dx;
                P = (eye(8) - K * H) * P;
            end
        end
        
        est_ned(k, :) = x(1:3)';
        P_trace(k) = P(1,1) + P(2,2);
        dop_log(k, :) = dop_curr;
    end
    
    [out_lat, out_lon, out_alt] = ned2geodetic(est_ned(:,1), est_ned(:,2), est_ned(:,3), ...
        ref_lat, ref_lon, ref_alt, wgs84);
    est_lla = [out_lat, out_lon, out_alt];
    
    fprintf('Kalman Filter Complete.\n');
end