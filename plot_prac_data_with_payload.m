%% Plot data from practical drone flight WITHOUT PAYLOAD
disp('start')

%% Load topics from csv into matrix
csv_folder = "/home/murray/Documents/QGroundControl/flight_report_folders/HoneyBee/HNB_2021-3-16_16-02-44_payload_follow_waypoints/csv";
log_name = "log_324_2021-3-16-16-02-44";

adc_report = readmatrix(strcat(csv_folder, '/', log_name, '_', 'adc_report', '_0.csv'));
estimator_status = readmatrix(strcat(csv_folder, '/', log_name, '_', 'estimator_status', '_0.csv'));

vehicle_odometry = readmatrix(strcat(csv_folder, '/', log_name, '_', 'vehicle_odometry', '_0.csv'));
vehicle_local_position = readmatrix(strcat(csv_folder, '/', log_name, '_', 'vehicle_local_position', '_0.csv'));
vehicle_local_position_setpoint = readmatrix(strcat(csv_folder, '/', log_name, '_', 'vehicle_local_position_setpoint', '_0.csv'));
disp('loaded csv files')

%% Payload angles
adc_time = adc_report(:,1)./1e6; % Timestamp of adc_report in seconds

alpha = adc_report(:,3+2); % forwards backwards payload angle (3+ to convert Channel_ID of adc_report to index)
beta  = adc_report(:,3+10); % side to side payload angle
payload_angles_ts   = timeseries([alpha, beta], adc_time, 'Name', 'Payload_angles'); % Time series of euler angles of drone

disp('payload time series')

%% States
state_time = estimator_status(:,1)./1e6; % Timestamp of state data in seconds

quaternions = estimator_status(:, (0:3) + 2); % +2 to use index from https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html
euler_angles = quat2eul(quaternions, 'XYZ'); % [X, Y, Z] angles in radians, using XYZ convention
attitude_ts   = timeseries(euler_angles, state_time, 'Name', 'Attitude'); % Time series of euler angles of drone

velocity = estimator_status(:,(4:6) + 2); % [dx, dy, dz]
velocity_ts   = timeseries(velocity, state_time, 'Name', 'Velocity'); % Time series of velocity

position = estimator_status(:,(7:9) + 2); % [x, y, z]
position_ts   = timeseries(position, state_time, 'Name', 'Position'); % Time series of position

% state_time = estimator_status(:,1)./1e6; % Timestamp of state data in seconds
% state_data = [euler_angles, velocity, position]; % State data in columns. Each coumn is a variable
% state_ts   = timeseries(state_data, state_time); % Time series of states

disp('state time series')

%% Setpoints
setpoint_time = vehicle_local_position_setpoint(:,1)./1e6; % Timestamp of in seconds

yaw_sp = vehicle_local_position_setpoint(:,5); % Yaw setpoint
yaw_sp_ts   = timeseries(yaw_sp, setpoint_time, 'Name', 'Yaw_setpoint'); % Time series of yaw setpoint

acc_sp = vehicle_local_position_setpoint(:, 10:12); % Acceleration setpoint
acc_sp_ts   = timeseries(acc_sp, setpoint_time, 'Name', 'Acceleration_setpoint'); % Time series of acceleration setpoint

disp('setpoints')

%% Plots
close all;

figure;
plot(payload_angles_ts);
legend('alpha', 'beta');

figure;
plot(attitude_ts);
legend('X', 'Y', 'Z');

figure;
plot(velocity_ts);
legend('X', 'Y', 'Z');

figure;
plot(position_ts);
legend('X', 'Y', 'Z');

figure;
plot(yaw_sp_ts);

figure;
plot(acc_sp_ts);
legend('X', 'Y', 'Z');

disp('plotted')