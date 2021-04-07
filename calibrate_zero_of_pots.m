%% Project payload angles to absolute coordinate frame 

disp('start')

%% Load topics from csv into matrix
load_csv_again = 1;
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

min_time = 80;
max_time = 180; % min of time range of constant hang

zero_row_indexes = (min_time <= adc_time)  &  (adc_time<=max_time);
adc_report = adc_report(zero_row_indexes, :);
adc_time = adc_time(zero_row_indexes);

%% Joystick attitude

% Convertion from adc value to radians
green_pot_line_fit = [ 0.038980944549164 -37.789860132384199]; % degrees linefit for polyval from calibration of pot connected to green wire
blue_pot_line_fit  = [ 0.018768173769117 -37.181837589261562];

green_adc2angle = @(adc) deg2rad(polyval(green_pot_line_fit, adc)); % Convert green adc value to angle [rad]
blue_adc2angle  = @(adc) deg2rad(polyval(blue_pot_line_fit,  adc)); % Convert green adc value to angle [rad]

% Define payload angle as euler angle, convention: 'XYZ'. 
% joystick x-axis connected to drone.
% joystick y-axis connected to x-axis
% z-axis does not matter
j_y = green_adc2angle(adc_report(:,3+4)); % [radians] Euler y angle of joystick. (side to side) (3+ to convert Channel_ID of adc_report to index)
j_x = blue_adc2angle(adc_report(:,3+10)); % [radians] Euler x angle of joystick. (forwards backwards)

offset_y = mean(j_y) % Offset to go to zero
offset_x = mean(j_x) % Offset to go to zero

j_y = j_y - offset_y; % Remove offset
j_x = j_x - offset_x; % Remove offset

%% Plots
close all;

figure;
plot(adc_time, rad2deg(j_x));
title('j_x');

figure;
plot(adc_time, rad2deg(j_y));
title('j_y');

disp('plotted')

