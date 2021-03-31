log_name = "green_10_deg_intervals";
topic = "adc_report";
directory = "/home/murray/OneDrive/Masters/QGroundControl/Logs/HoneyBee/2021-03-29_calibrate_pot_green";
filename = strcat(directory, "/", log_name, "_", topic, "_0", ".csv");
csv_matrix = readmatrix(filename);

disp('start')

timestamp = csv_matrix(:,1);
raw_data = csv_matrix(:,3:14);

close all;
channels = [4,10,11]; % Chaneel 11 = 3.3V, Channel 4 = 6.6V
for i = channels
    figure(i+1);
    pause
    plot(timestamp, raw_data(:,i+1))
end

figure;
plot(timestamp, raw_data)
ids_string = {};

for i = channels
    ids_string = [ids_string, {int2str(i)}];
end
legend('Forward back', 'Side to side')


disp('end')