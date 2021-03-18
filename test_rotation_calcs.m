

uav_quat = eul2quat(deg2rad([10 0 0]), 'XYZ'); % Quaternions of uav +2 to use index from https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html

joy_euler = deg2rad([5, 0, 0]); % Euler angles of joystick
joy_quat = eul2quat(joy_euler, 'XYZ');


payload_abs_rot = quatmultiply(uav_quat,joy_quat); % absolute rotation of payload. First joystick rotation. Then UAV attitude rotation
payload_vector = quatrotate(payload_abs_rot, [0 0 1]); % Rotate neutral hanging payload by joystick angle, then attitude

x_angle_from_z = rad2deg(atan2(payload_vector(2),payload_vector(3)))