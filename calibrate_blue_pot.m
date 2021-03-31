%% Blue
log_name = "green_10_deg_intervals";
topic = "adc_report";
directory = "/home/murray/OneDrive/Masters/QGroundControl/Logs/HoneyBee/2021-03-29_calibrate_pot_green";
filename = strcat(directory, "/", log_name, "_", topic, "_0", ".csv");
csv_matrix = readmatrix(filename);

timestamp = csv_matrix(:,1).*1e-6; % in seconds
raw_data = csv_matrix(:,3:14);

% close all;
% channels = [10]; % Channel 11 = 3.3V, Channel 4 = 6.6V
% for i = channels
%     figure(i+1);
%     plot(timestamp, raw_data(:,i+1))
% end

% Manual measurements
% Take stick to angle on protractor
% send command on MAVlink console: listener adc_report -n 3
% channel 10
manual_input = [... % [angle degrees, adc 10, adc 11] 
    30, 8;
    40, 9;
    50, 10;
    55, 302;
    60, 878;
    70, 1250;
    80, 1426;
    90, 1412;
    100, 1638;
    110, 1725;
    120, 1777;
    130, 1868;
    140, 1870;
    150, 1854];

manual_input(:,1) = manual_input(:,1) - 90; % Convert to 0 deg = down

ch = 2; % 2 = ch 10, 3 = ch 11

% pot range = [-]
adc_line_fit = polyfit(manual_input(:,ch), manual_input(:,1), 1);
adc2angle = @(adc) polyval(adc_line_fit, adc); % Convert green adc value to angle [degrees]

angle_line_fit = polyfit(manual_input(:,1), manual_input(:,ch), 1);
angle2adc = @(angle) polyval(angle_line_fit, angle); % Convert green adc value to angle [degrees]

% Plot range
figure
plot(manual_input(:,1), manual_input(:,ch), 'x')
hold on
x_range = linspace(min(manual_input(:,1)), max(manual_input(:,1)), 100);
plot(x_range, angle2adc(x_range))
title(['green pot manual readings, ch = ', num2str(ch)])
xlabel('adc reading')
ylabel('Angle [degrees]')
