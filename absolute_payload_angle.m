%% Project payload angles to absolute coordinate frame 

disp('start')

%% Load topics from csv into matrix
load_csv_again = 0;
if load_csv_again
    [ulog_name,csv_folder] = uigetfile('/home/esl/Masters/QGroundControl/Logs/HoneyBee/*.ulg', 'Choose ulog file to access') % GUI to choose ulog file. % [ulog filename, path to folder with csv files]
    ulog_name = erase(ulog_name, '.ulg'); % remove file extention

    % csv_folder = "/home/esl/Masters/QGroundControl/Logs/HoneyBee/2021-04-06_check_pots_in_lab";
    % log_name = "log_354_2021-4-6-17-03-48";

    adc_report = readmatrix(strcat(csv_folder, ulog_name, '_', 'adc_report', '_0.csv'));
    estimator_status = readmatrix(strcat(csv_folder, ulog_name, '_', 'estimator_status', '_0.csv'));

    disp('loaded csv files')
end
%% Time matching
% have common time series so no extrapolation occurs between timeseries

state_time = estimator_status(:,1)./1e6; % Timestamp of state data in seconds
adc_time = adc_report(:,1)./1e6; % Timestamp of adc_report in seconds

max_time = min([max(adc_time), max(state_time)]); % get max value in time array that overlaps in both timeseries
min_time = max([min(adc_time), min(state_time)]); % get min value in time array that overlaps in both timeseries
basis_time = state_time; % Use state_time as basis.
combo_time = basis_time(  (min_time <= basis_time)  &  (basis_time <= max_time)  ); % Remove values outside overlapping range

disp("time matching done")

%% States

uav_quat    = estimator_status(:, (0:3) + 2); % Quaternions of uav +2 to use index from https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html
uav_quat_ts = timeseries(uav_quat, state_time, 'Name', 'Attitude'); % Time series of euler angles of drone
uav_quat_ts = resample(uav_quat_ts, combo_time, 'linear'); % Resample for matching time sequence
uav_quat    = uav_quat_ts.Data; % Data from resampled timeseries

disp('state time series')

%% Attitude
euler_angles = rad2deg(quat2eul(uav_quat, 'ZYX')); % [X, Y, Z] angles in radians, using XYZ convention
euler_angles_ts = timeseries(euler_angles, combo_time, 'Name', 'euler_angles'); % Time series of euler angles of drone

axang = quat2axang(uav_quat); % Axis angle representation of attitude
axang_ts   = timeseries(axang, combo_time, 'Name', 'axang'); % Time series of axis angle attitude of drone


%% Joystick attitude

% Convertion from adc value to radians
green_pot_line_fit = [ 0.038980944549164 -37.789860132384199]; % degrees linefit for polyval from calibration of pot connected to green wire
blue_pot_line_fit  = [ 0.018768173769117 -37.181837589261562];

offset_y = 0.03; %-0.033242678592147; % [degrees] Offset calculated afterwards
offset_x = -0.025; % -0.037739964002411; % [degrees] Offset calculated afterwards

green_adc2angle = @(adc) -deg2rad(polyval(green_pot_line_fit, adc)) - offset_y; % Convert green adc value to angle [rad]
blue_adc2angle  = @(adc) deg2rad(polyval(blue_pot_line_fit,  adc)) - offset_x; % Convert green adc value to angle [rad]

% Define payload angle as euler angle, convention: 'XYZ'. 
% joystick x-axis connected to drone.
% joystick y-axis connected to x-axis
% z-axis does not matter
j_y = green_adc2angle(adc_report(:,3+4)); % [radians] Euler y angle of joystick. (side to side) (3+ to convert Channel_ID of adc_report to index)
j_x = blue_adc2angle(adc_report(:,3+10)); % [radians] Euler x angle of joystick. (forwards backwards)
j_z = zeros(size(j_y)); % No z angle
joy_euler = [j_z, j_y, j_x]; % Euler angles of joystick % MATLAB euler format is [z, y, x]

joy_quat    = eul2quat(joy_euler, 'XYZ');
joy_quat_ts = timeseries(joy_quat, adc_time, 'Name', 'Attitude'); % Time series of euler angles of drone

joy_quat_ts = resample(joy_quat_ts, combo_time, 'linear'); % Resample for matching time sequence
joy_quat    = joy_quat_ts.Data; % Data from resampled timeseries

disp('joystick time series')

%% Payload attitude
payload_abs_rot = quatmultiply(uav_quat,joy_quat); % absolute rotation of payload. First joystick rotation. Then UAV attitude rotation
payload_vector  = quatrotate(quatinv(payload_abs_rot), [0 0 1]); % unit vector representing direction of payload. Rotate neutral hanging payload by joystick angle, then attitude. % "quatrotate" rotates the coordinate frame, not the vector, therefore use inverse in function (https://www.mathworks.com/matlabcentral/answers/465053-rotation-order-of-quatrotate)
payload_vector_angle_x = -atan2(payload_vector(:,2), payload_vector(:,3)); % [radians] absolute angle of payload vector from z axis, about the x axis, projected on yz plane. NOT euler angle. negative, becasue +y gives negative rotation about x
payload_vector_angle_y =  atan2(payload_vector(:,1), payload_vector(:,3)); % [radians] absolute angle of payload vector from z axis, about the y axis, projected on xz plane. NOT euler angle

payload_vector_angles = [payload_vector_angle_x, payload_vector_angle_y]; % [radians] [x, y] absolute angle of payload vector. NOT euler angles

%% Plots
close all;

% figure;
% plot(combo_time, uav_quat);
% title('uav_quat');
% 
% figure;
% plot(combo_time, joy_quat);
% title('joy_quat');

figure;
plot(combo_time, rad2deg(payload_vector));
legend('x', 'y', 'z');
title('payload_vector');

figure;
plot(combo_time, rad2deg(payload_vector_angles));
legend('x', 'y');
title('payload_vector_angles');
% 
%%
figure;
plot(adc_time, (j_x));
title('j_x');

figure;
plot(adc_time, (j_y));
title('j_y');
% 
% %%
% figure;
% title('euler angles');
% plot(euler_angles_ts);
% legend('X', 'Y', 'Z');
% 
% figure;
% title('axang');
% plot(axang_ts);
% legend('X', 'Y', 'Z', 'theta');
% 
% figure;
% title('axang');
% plot(axang_ts);
% legend('X', 'Y', 'Z', 'theta');

% 
% figure;
% plot(velocity_ts);
% legend('X', 'Y', 'Z');
% 
% figure;
% plot(position_ts);
% legend('X', 'Y', 'Z');
% 
% figure;
% plot(yaw_sp_ts);
% 
% figure;
% plot(acc_sp_ts);
% legend('X', 'Y', 'Z');

disp('plotted')

