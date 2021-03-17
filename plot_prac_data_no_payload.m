%% Plot data from practical drone flight WITHOUT PAYLOAD
disp('start')

%% Load topics from csv into matrix
csv_folder = "/home/murray/Documents/QGroundControl/flight_report_folders/HoneyBee/HNB_2021-3-16_15-45-54_manual_no_payload/csv";
log_name = "log_319_2021-3-16-15-45-54";

adc_report = readmatrix(strcat(csv_folder, '/', log_name, '_', 'adc_report', '_0.csv'));
estimator_status = readmatrix(strcat(csv_folder, '/', log_name, '_', 'estimator_status', '_0.csv'));

vehicle_odometry = readmatrix(strcat(csv_folder, '/', log_name, '_', 'vehicle_odometry', '_0.csv'));
vehicle_local_position = readmatrix(strcat(csv_folder, '/', log_name, '_', 'vehicle_local_position', '_0.csv'));
vehicle_local_position_setpoint = readmatrix(strcat(csv_folder, '/', log_name, '_', 'vehicle_local_position_setpoint', '_0.csv'));
disp('loaded csv files')

%% Create time series
state_time = estimator_status(:,1)./1e6; % Timestamp of state data in seconds

quaternions = estimator_status(:, (0:3) + 2); % +2 to use index from https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html
euler_angles = quat2eul(quaternions, 'XYZ'); % [X, Y, Z] angles in radians, using XYZ convention
attitude_ts   = timeseries(euler_angles, state_time, 'Name', 'Attitude'); % Time series of euler angles of drone

velocity = estimator_status(:,4:6 + 2); % [dx, dy, dz]
velocity_ts   = timeseries(velocity, state_time, 'Name', 'Velocity'); % Time series of velocity

position = estimator_status(:,7:9 + 2); % [x, y, z]
position_ts   = timeseries(position, state_time, 'Name', 'Position'); % Time series of position

% state_time = estimator_status(:,1)./1e6; % Timestamp of state data in seconds
% state_data = [euler_angles, velocity, position]; % State data in columns. Each coumn is a variable
% state_ts   = timeseries(state_data, state_time); % Time series of states

disp('created time series')

%% Plots
close all;

figure;
title('euler angles');
plot(attitude_ts);
legend('X', 'Y', 'Z');

figure;
title('velocity');
plot(velocity_ts);
legend('X', 'Y', 'Z');

figure;
title('position');
plot(position_ts);
legend('X', 'Y', 'Z');

disp('plotted')
% % Input data
% input_time = position_setpoint_triplet(:,42)./1e6; % current.timestamp. Timestamp of input data in seconds
% input_data = position_setpoint_triplet(:,45:47); % current.x , .y , .z. % Input data in columns. Each coumn is a variable
% 
% input_offset = mean(input_data,1); % Input needed to keep at a fixed points ??? Should this not be zero?
% input_data  = input_data - input_offset; % Adjust for unmeasured input
% 
% input_ts   = timeseries(input_data, input_time); % Time series of input data


