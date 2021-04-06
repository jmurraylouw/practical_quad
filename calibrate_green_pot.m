%% Green
% log_name = "green_10_deg_intervals";
% topic = "adc_report";
% directory = "/home/murray/OneDrive/Masters/QGroundControl/Logs/HoneyBee/2021-03-29_calibrate_pot_green";
% filename = strcat(directory, "/", log_name, "_", topic, "_0", ".csv");
% csv_matrix = readmatrix(filename);
% 
% timestamp = csv_matrix(:,1).*1e-6; % in seconds
% raw_data = csv_matrix(:,3:14);
% 
% close all;
% channels = [4]; % Chaneel 11 = 3.3V, Channel 4 = 6.6V
% for i = channels
%     figure(i+1);
%     plot(timestamp, raw_data(:,i+1))
% end

% Manual measurements
% Take stick to angle on protractor
% send command on MAVlink console: listener adc_report -n 3
manual_input = [... % [angle degrees, adc]
    55, 81;    
    60, 209;
    70, 480;
    80, 737;
    90, 941;
    100, 1197;
    110, 1461;
    120, 1728;
    125, 1898;
    55, 82;
    60, 209;
    70, 458;
    80, 698;
    90, 925;
    100, 1214;
    110, 1473;
    120, 1751;
    125, 1908
];

manual_input(:,1) = manual_input(:,1) - 90; % Convert to 0 deg = down

% pot range = [-]
green_pot_line_fit = polyfit(manual_input(:,2), manual_input(:,1), 1);

green_adc2angle = @(adc) polyval(green_pot_line_fit, adc); % Convert green adc value to angle [degrees]

angle = green_adc2angle(2)

% Plot range
figure
plot(manual_input(:,2), manual_input(:,1), 'x')
hold on
plot(manual_input(:,2), green_adc2angle(manual_input(:,2)))
title('green pot manual readings')
xlabel('adc reading')
ylabel('Angle [degrees]')
