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
    50, 40, 26;
    50, 44, 24;
    50, 38, 27;
    60, 2130, 1391;
    60, 2133, 1418;
    60, 2146, 1400;
    70, 2995, 2117;
    70, 2983, 2116;
    70, 2985, 2112;
    80, 3302, 2356;
    80, 3304, 2355;
    80, 3288, 2348;
    90, 3033, 2156;
    90, 3081, 2158;
    90, 3131, 2196;
    100, 3575, 2575;
    100, 3578, 2561;
    100, 3578, 2576;
    110, 3696, 2660;
    110, 3703, 2654;
    110, 3703, 2662;
    120, 3709, 2664;
    120, 3719, 2669;
    120, 3717, 2659;
    130, 3794, 2694;
    130, 3795, 2714;
    130, 3796, 2692];

manual_input(:,1) = manual_input(:,1) - 90; % Convert to 0 deg = down

ch = 2; % 2 = ch 10, 3 = ch 11

% pot range = [-]
line_fit = polyfit(manual_input(:,ch), manual_input(:,1), 2);

adc2angle = @(adc) polyval(line_fit, adc); % Convert green adc value to angle [degrees]

angle = adc2angle(0)

% Plot range
figure
plot(manual_input(:,ch), manual_input(:,1), 'x')
hold on
x_range = linspace(min(manual_input(:,ch)), max(manual_input(:,ch)), 100);
plot(x_range, adc2angle(x_range))
title(['green pot manual readings, ch = ', num2str(ch)])
xlabel('adc reading')
ylabel('Angle [degrees]')
