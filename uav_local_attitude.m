%% Project payload angles to absolute coordinate frame 

disp('start')

%% Functions
quat_rot_vect = @(vect, quat) quatrotate(quatinv(quat), vect); % Rotates vector by quaternion % built in "quatrotate" rotates the coordinate frame, not the vector, therefore use inverse in function (https://www.mathworks.com/matlabcentral/answers/465053-rotation-order-of-quatrotate)

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


%% Attitude

uav_quat    = estimator_status(:, (0:3) + 2); % Quaternions of uav +2 to use index from https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html
uav_quat_ts = timeseries(uav_quat, state_time, 'Name', 'Attitude'); % Time series of euler angles of drone
uav_quat_ts = resample(uav_quat_ts, combo_time, 'linear'); % Resample for matching time sequence
uav_quat    = uav_quat_ts.Data; % Data from resampled timeseries


%% Remove Z of uav attitude

heading = quat2heading(uav_quat);
quat_inv_heading = quatinv(eul2quat([heading, zeros(size(heading)), zeros(size(heading))])); % inverse of quat of heading 

uav_quat = quatmultiply(quat_inv_heading, uav_quat); % Remove heading

%% UAV into vector form
uav_vector  = quat_rot_vect([0 0 1], uav_quat); % unit vector representing direction of payload. Rotate neutral hanging payload by joystick angle, then attitude. % "quatrotate" rotates the coordinate frame, not the vector, therefore use inverse in function (https://www.mathworks.com/matlabcentral/answers/465053-rotation-order-of-quatrotate)
uav_vector_angle_x = -atan2(uav_vector(:,2), uav_vector(:,3)); % [radians] absolute angle of payload vector from z axis, about the x axis, projected on yz plane. NOT euler angle. negative, becasue +y gives negative rotation about x
uav_vector_angle_y =  atan2(uav_vector(:,1), uav_vector(:,3)); % [radians] absolute angle of payload vector from z axis, about the y axis, projected on xz plane. NOT euler angle

uav_vector_angles = [uav_vector_angle_x, uav_vector_angle_y]; % [radians] [x, y] absolute angle of payload vector. NOT euler angles

disp('state time series')


%% Plots
close all;

figure;
plot(combo_time, rad2deg(heading));
title('heading');
% 
% figure;
% plot(combo_time, joy_quat);
% title('joy_quat');

figure;
plot(combo_time, (uav_vector));
legend('x', 'y', 'z');
title('uav_vector');

% figure;
% plot(combo_time, rad2deg(uav_vector_angles));
% legend('x', 'y', 'z');
% title('uav_vector_angles');


