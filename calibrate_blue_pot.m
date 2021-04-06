%% Blue
% log_name = "log_347_2021-3-31-16-09-42";
% topic = "adc_report";
% directory = "/home/murray/OneDrive/Masters/QGroundControl/Logs/HoneyBee/2021-03-29_calibrate_pot_blue";
% filename = strcat(directory, "/", log_name, "_", topic, "_0", ".csv");
% csv_matrix = readmatrix(filename);
% 
% timestamp = csv_matrix(:,1).*1e-6; % in seconds
% raw_data = csv_matrix(:,3:14);
% 
% close all;
% channels = [4,10]; % Channel 10 = 3.3V, Channel 4 = 6.6V
% for i = channels
%     figure(i+1);
%     plot(timestamp, raw_data(:,i+1)) % +1 to convert channel ID to MATLAB index
% end

% Manual measurements
% Take stick to angle on protractor
% send command on MAVlink console: listener adc_report -n 3
% channel 10
% pot range = -35 to 35 degrees
manual_input = [... % [angle degrees, adc 10, adc 11] 
    55, 116, 68;    
    60, 361, 216;
    70, 916, 555;
    80, 1458, 890;
    90, 1987, 1281;
    100, 2545, 1741;
    110, 3075, 2160;
    120, 3588, 2570;
    125, 3796, 2725;
    55, 128, 76;
    60, 379, 228;
    70, 882, 526;
    80, 1447, 868;
    90, 1970, 1235;
    100, 2539, 1689;
    110, 3109, 2131;
    120, 3559, 2534;
    125, 3805, 2716];

manual_input(:,1) = manual_input(:,1) - 90; % Convert to 0 deg = down

ch = 2; % ch = 2 has least noise. choose which 3.3V report channel. 2 = ch 10, 3 = ch 11

blue_pot_line_fit = polyfit(manual_input(:,ch), manual_input(:,1), 1); % line fit for adc2angle
blue_adc2angle = @(adc) polyval(blue_pot_line_fit, adc); % Convert green adc value to angle [degrees]
angle = blue_adc2angle(71)

angle_line_fit = polyfit(manual_input(:,1), manual_input(:,ch), 1); % line fit for angle2adc
angle2adc = @(angle) polyval(angle_line_fit, angle); % Convert green adc value to angle [degrees]

% Plot range
figure
plot(manual_input(:,1), manual_input(:,ch), 'x')
hold on
x_range = linspace(min(manual_input(:,1)), max(manual_input(:,1)), 100);
plot(x_range, angle2adc(x_range))
title(['blue pot manual readings, ch = ', num2str(ch)])
ylabel('adc reading')
xlabel('Angle [degrees]')
