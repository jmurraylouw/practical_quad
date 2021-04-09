
quat_rot_vect = @(vect, quat) quatrotate(quatinv(quat), vect); % Rotates vector by quaternion % built in "quatrotate" rotates the coordinate frame, not the vector, therefore use inverse in function (https://www.mathworks.com/matlabcentral/answers/465053-rotation-order-of-quatrotate)

% Euler angles
x = deg2rad(30);
y = deg2rad(20);
z = deg2rad(10);

vect = [0 0 1];

euler_angles = [z, y, x];
quat = eul2quat(euler_angles, 'ZYX')
quat_ans = quat_rot_vect(vect, quat) % Rotate vector by quaternion

rot_ans - quat_ans

payload_vector_angle_x = -atan2(quat_ans(:,2), quat_ans(:,3)); % [radians] absolute angle of payload vector from z axis, about the x axis, projected on yz plane. NOT euler angle. negative, becasue +y gives negative rotation about x
payload_vector_angle_y =  atan2(quat_ans(:,1), quat_ans(:,3)); % [radians] absolute angle of payload vector from z axis, about the y axis, projected on xz plane. NOT euler angle

payload_vector_angles = [payload_vector_angle_x, payload_vector_angle_y]; % [radians] [x, y] absolute angle of payload vector. NOT euler angles
