%% Project payload angles to absolute coordinate frame 

disp('start')

%% Load topics from csv into matrix
csv_folder = "/home/esl/Masters/QGroundControl/flight_report_folders/HoneyBee/HNB_2021-3-16_16-02-44_payload_follow_waypoints/csv";
log_name = "log_324_2021-3-16-16-02-44";

adc_report = readmatrix(strcat(csv_folder, '/', log_name, '_', 'adc_report', '_0.csv'));
estimator_status = readmatrix(strcat(csv_folder, '/', log_name, '_', 'estimator_status', '_0.csv'));

disp('loaded csv files')

%% States
state_time = estimator_status(:,1)./1e6; % Timestamp of state data in seconds

uav_quat = estimator_status(:, (0:3) + 2); % Quaternions of uav +2 to use index from https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html
uav_quat_ts   = timeseries(uav_quat, state_time, 'Name', 'Attitude'); % Time series of euler angles of drone

disp('state time series')

%% Joystick attitude

adc_time = adc_report(:,1)./1e6; % Timestamp of adc_report in seconds

% Convertion from adc value to radians
green_pot_line_fit = [ 0.038980944549164 -37.789860132384199]; % linefit for polyval from calibration of pot connected to green wire
blue_pot_line_fit = [ 0.018768173769117 -37.181837589261562];

green_adc2angle = @(adc) polyval(green_line_fit, adc); % Convert green adc value to angle [degrees]


% Define payload angle as euler angle, convention: 'XYZ'. 
% joystick x-axis connected to drone.
% joystick y-axis connected to x-axis
% z-axis does not matter
j_x = adc_report(:,3+2); % [radians] Euler x angle of joystick. (side to side) (3+ to convert Channel_ID of adc_report to index)
j_y = adc_report(:,3+10); % [radians] Euler y angle of joystick. (forwards backwards)
j_z = zeros(size(j_y)); % No z angle
joy_euler = [j_x, j_y, j_z]; % Euler angles of joystick

joy_quat = eul2quat(joy_euler, 'ZYX');
joy_quat_ts = timeseries(joy_quat, adc_time, 'Name', 'Attitude'); % Time series of euler angles of drone
joy_quat_ts = resample(joy_quat_ts, state_time, 'linear'); % Resample to match state time

disp('joystick time series')

%% Payload attitude
payload_abs_rot = quatmultiply(uav_quat,joy_quat); % absolute rotation of payload. First joystick rotation. Then UAV attitude rotation
payload_vector = quatrotate(payload_abs_rot, [0 0 1]); % Rotate neutral hanging payload by joystick angle, then attitude

%% ??? test this method with known rotations
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

