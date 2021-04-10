
%% Project payload angles to absolute coordinate frame 

disp('start')

rng(1)
%% Attitude
z = deg2rad(randn*10*2525264225);
y = deg2rad(randn*10);
x = deg2rad(randn*10);

uav_quat    = eul2quat([z, y, x]);
uav_vector  = quat_rot_vect([0 0 1], uav_quat); % unit vector representing direction of payload. Rotate neutral hanging payload by joystick angle, then attitude. % "quatrotate" rotates the coordinate frame, not the vector, therefore use inverse in function (https://www.mathworks.com/matlabcentral/answers/465053-rotation-order-of-quatrotate)

%% Remove Z of uav attitude

quat_rot_vect = @(vect, quat) quatrotate(quatinv(quat), vect); % Rotates vector by quaternion % built in "quatrotate" rotates the coordinate frame, not the vector, therefore use inverse in function (https://www.mathworks.com/matlabcentral/answers/465053-rotation-order-of-quatrotate)
heading = quat2heading(uav_quat);
quat_inv_heading = quatinv(eul2quat([heading, 0, 0])); % inverse of quat of heading 

%% Joystick attitude
z = deg2rad(0);
y = deg2rad(randn*10);
x = deg2rad(randn*10);

joy_euler = [z, y, x];

joy_quat    = eul2quat(joy_euler, 'XYZ');

%% Payload attitude

payload_abs_rot = quatmultiply(uav_quat, joy_quat); % Attitude of payload in World frame. First joystick rotation. Then UAV attitude rotation
payload_abs_rot = quatmultiply(quat_inv_heading, payload_abs_rot); % Attitude of payload in Loacl frame (world with local heading). World frame, then inverse heading

payload_vector  = quatrotate(quatinv(payload_abs_rot), [0 0 1]) % unit vector representing direction of payload. Rotate neutral hanging payload by joystick angle, then attitude. % "quatrotate" rotates the coordinate frame, not the vector, therefore use inverse in function (https://www.mathworks.com/matlabcentral/answers/465053-rotation-order-of-quatrotate)



