disp('start')

%% Green
log_name = "green_pot_mid_right_left";
topic = "adc_report";
directory = "/home/murray/OneDrive/Masters/QGroundControl/Logs/HoneyBee/2021-03-29_calibrate_pot_green";
filename = strcat(directory, "/", log_name, "_", topic, "_0", ".csv");
csv_matrix = readmatrix(filename);

timestamp = csv_matrix(:,1).*1e-6; % in seconds
raw_data = csv_matrix(:,3:14);

close all;
channels = [4,11]; % Chaneel 11 = 3.3V, Channel 4 = 6.6V
for i = channels
    figure(i+1);
    plot(timestamp, raw_data(:,i+1))
end

% Calibration values:
green_adc   = [1911; 903]; % Value read by ADC
green_angle = [  90;   0]; % Angle of calibration [degrees]. If looking at pot, right is 90, left -90, middle 0 deg

line_fit = polyfit(green_adc, green_angle, 1);

green_adc2angle = @(adc) polyval(line_fit, adc); % Convert green adc value to angle [degrees]

angle = green_adc2angle(0)

%% Green
log_name = "blue_pot_mid_right_left";
topic = "adc_report";
directory = "/home/murray/OneDrive/Masters/QGroundControl/Logs/HoneyBee/2021-03-29_calibrate_pot_blue";
filename = strcat(directory, "/", log_name, "_", topic, "_0", ".csv");
csv_matrix = readmatrix(filename);

timestamp = csv_matrix(:,1).*1e-6; % in seconds
raw_data = csv_matrix(:,3:14);

close all;
channels = [4,11]; % Chaneel 11 = 3.3V, Channel 4 = 6.6V
for i = channels
    figure(i+1);
    plot(timestamp, raw_data(:,i+1))
end

% Calibration values:
blue_adc   = [1911; 903]; % Value read by ADC
blue_angle = [  90;   0]; % Angle of calibration [degrees]

line_fit = polyfit(blue_adc, blue_angle, 1);

blue_adc2angle = @(adc) polyval(line_fit, adc); % Convert green adc value to angle [degrees]

angle = blue_adc2angle(0)

disp('end')

