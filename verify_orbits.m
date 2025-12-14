%% Verification
clear; clc; close all;
addpath('Utils');

% 1. Configuration
tleFile = 'Data/TLE/gps_2011_active.tle'; % USE THE STRICT FILE
startTime = datetime(2011, 9, 26, 13, 3, 0, 'TimeZone', 'UTC');
lat = 49.0112; 
lon = 8.4228;
alt = 112; 

%% 2. Setup Scenario
fprintf('Initializing Scenario...\n');
sc = satelliteScenario(startTime, startTime + minutes(1), 60);
sats = satellite(sc, tleFile); 
car = groundStation(sc, lat, lon, 'Altitude', alt, 'Name', 'KITTI_Car');

% Check integrity - Print position of first satellite
[p, v] = states(sats(1), startTime, 'CoordinateFrame', 'geographic');
fprintf('DEBUG: Sat #1 is at Lat: %.2f, Lon: %.2f, Alt: %.2f km\n', ...
    p(1), p(2), p(3)/1000);

%% 3. Calculate Visibility
[az, el, r] = aer(car, sats, startTime);

visible_idx = el > 0;
vis_az = az(visible_idx);
vis_el = el(visible_idx);
vis_names = {sats(visible_idx).Name}; 

fprintf('Satellites actually above horizon: %d\n', length(vis_names));

%% 4. Generate Skyplot
if isempty(vis_names)
    error('Still no satellites visible! Check TLE file or Time.');
end

figure('Name', 'GPS Skyplot', 'Color', 'w');
sp = skyplot(vis_az, vis_el); 

% FIX FOR THE ERROR: Convert Cell of Char to String Array
sp.LabelData = string(vis_names); 

title({'GPS Constellation over Karlsruhe', datestr(startTime)});