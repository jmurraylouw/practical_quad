
disp('start')

log_name = "log_319_2021-3-16-10-06-40";
directory = "/home/murray/Documents/QGroundControl/Logs/HoneyBee/2021-03-16_Check_sensors_in_lab/319";

mag0 = readmatrix(strcat(directory, "/", log_name, "_sensor_mag_0", ".csv"));
mag1 = readmatrix(strcat(directory, "/", log_name, "_sensor_mag_1", ".csv"));

mag0_ts = timeseries(mag0(:,4:6), mag0(:,1));
mag1_ts = timeseries(mag1(:,4:6), mag1(:,1));

close all;
plot(mag0_ts)

hold on;
plot(mag1_ts)
legend('x0', 'y0', 'z0',    'x1', 'y1', 'z1')
disp('end')

figure
plot(mag0_ts.Time, mag0_ts.Data(:,1))
hold on;
plot(mag1_ts.Time, mag1_ts.Data(:,1))
legend('mag0', 'mag1')
