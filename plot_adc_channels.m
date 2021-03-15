log_name = "log_301_2021-3-5-13-55-12_adc_report_0";
directory = "/home/murray/Documents/QGroundControl/HoneyBee flight logs/HNB_2021-03-05_13-55_short_cable_small_weight";
filename = strcat(directory, "/", log_name, ".csv");
csv_matrix = readmatrix(filename);

disp('start')

timestamp = csv_matrix(:,1);
raw_data = csv_matrix(:,3:14);

close all;
channels = [4,11]; % Chaneel 11 = 3.3V, Channel 4 = 6.6V
for i = channels
    figure(i+1);
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