%% Implentation of HAVOK on data from practical HoneyBee flight data
% close all;
disp('start')

% Load topics from csv into matrix
csv_folder = "/home/murray/Documents/QGroundControl/Logs/HoneyBee/2021-02-26_Smokey_day_baro_test_flights/csv";
log_name = "log_73_2021-2-26-14-53-08";

% adc_report = readmatrix(strcat(csv_folder, '/', log_name, '_', 'adc_report', '_0.csv'));
estimator_status = readmatrix(strcat(csv_folder, '/', log_name, '_', 'estimator_status', '_0.csv'));
position_setpoint_triplet = readmatrix(strcat(csv_folder, '/', log_name, '_', 'position_setpoint_triplet', '_0.csv'));

%% figure out timestamp sync!!!!

% Payload angles
% alpha = adc_report(:,3+3); % forwards backwards payload angle (3+ to convert Channel_ID of adc_report to index)
% beta  = adc_report(:,3+10); % side to side payload angle

% Convert quaternions to euler angles
quaternions = estimator_status(:,0:3 + 2); % +2 to use index from https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html
euler_angles = quat2eul(quaternions); % [X, Y, Z], using ZYX convention

% plot(euler_angles);
% legend('X', 'Y', 'Z');

% Velocity
velocity = estimator_status(:,4:6 + 2); % [dx, dy, dz]

% Position
position = estimator_status(:,7:9 + 2); % [x, y, z]

% Populate matrix with state vector for every timestamp
state_time = estimator_status(:,1)./1e6; % Timestamp of state data in seconds
state_data = [euler_angles, velocity, position]; % State data in columns. Each coumn is a variable
state_ts   = timeseries(state_data, state_time); % Time series of states

% Input data
input_time = position_setpoint_triplet(:,42)./1e6; % current.timestamp. Timestamp of input data in seconds
input_data = position_setpoint_triplet(:,45:47); % current.x , .y , .z. % Input data in columns. Each coumn is a variable

input_offset = mean(input_data,1); % Input needed to keep at a fixed points ??? Should this not be zero?
input_data  = input_data - input_offset; % Adjust for unmeasured input

input_ts   = timeseries(input_data, input_time); % Time series of input data

% Data dimentions
nx = size(state_data,2); % number of states
nu = size(input_data,2); % number of measurements

% HAVOK

% Ts = 0.03;     % Desired sample time
% Ts_havok = Ts;
% 
% % Adjust for constant disturbance / mean control values
% % u_bar = mean(out.u.Data,1); % Input needed to keep at a fixed point

% out.u.Data  = out.u.Data - u_bar; % Adjust for unmeasured input

% Training data
% train_time = 0:Ts:300;
% x_train = resample(out.x, train_time );% Resample time series to desired sample time and training period  
% u_train = resample(out.u, train_time );  
% t_train = x_train.Time';
% N_train = length(t_train);
% 
% x_train = x_train.Data';
% y_train = x_train(y_rows,:);
% u_train = u_train.Data';
% 
% % Testing data
% % test_time = 400:Ts:500;
% x_test = resample(out.x, test_time );  
% u_test = resample(out.u, test_time );  
% t_test = x_test.Time';
% N_test = length(t_test); % Num of data samples for testing
% 
% x_test = x_test.Data';
% y_test = x_test(y_rows,:); % One sample of testing data overlaps for initial condition
% u_test = u_test.Data';
% 

% comment = ''; % Extra comment to differentiate this run
% 
% % Read previous results
% sigma = 0;
% sig_str = strrep(num2str(sigma),'.','_'); % Convert sigma value to string
results_file = ['Data/havok_results_', comment, simulation_data_file, '_sig=', sig_str, '.mat'];

try
    load(results_file);
    results(~results.q,:) = []; % remove empty rows
    
    % Parameters
    best_row = find(results.MAE_mean == min(results.MAE_mean));
    best_results = results(best_row,:);
    q = double(best_results.q);
    p = double(best_results.p);
    
    only_q_Ts = 0; % Try best result for specific q
    if only_q_Ts
        q = 39;
        q_results = results((results.q == q & results.Ts == Ts),:);
        best_row = find(q_results.MAE_mean == min(q_results.MAE_mean));
        best_results = q_results(best_row,:)
        p = double(best_results.p);
    end
    
    override = 0;
    if override
        '!!!!!Override!!!!!!!'
        p = 40
%         q =20
    end
    % % Override parameters:
    % q = 80
    % p = 40
   
    q
    p
    
catch
    disp('No saved results file')  
end

w = N_train - q + 1; % num columns of Hankel matrix
D = (q-1)*Ts; % Delay duration (Dynamics in delay embedding)

% Create Hankel matrix with measurements
Y = zeros((q)*ny,w); % Augmented state Y[k] at top
for row = 0:q-1 % Add delay coordinates
    Y((end - ny*(row+1) + 1):(end - ny*row), :) = y_train(:, row + (1:w));
end

Upsilon = u_train(:, q:end); % Leave out last time step to match V_til_1
YU_bar = [Y; Upsilon];

% SVD of the Hankel matrix
[U1,S1,V1] = svd(YU_bar, 'econ');
figure, semilogy(diag(S1), 'x'), hold on;
title('Singular values of Omega, showing p truncation')
plot(p, S1(p,p), 'ro'), hold off;

% Truncate SVD matrixes
U_tilde = U1(:, 1:p); 
S_tilde = S1(1:p, 1:p);
V_tilde = V1(:, 1:p);

% Setup V2 one timestep into future from V1
V_til_2 = V_tilde(2:end  , :)'; % Turnd on side (wide short matrix)
V_til_1 = V_tilde(1:end-1, :)';

% DMD on V
AB_tilde = V_til_2*pinv(V_til_1); % combined A and B matrix, side by side
AB_tilde = stabilise(AB_tilde,3);

% Convert to x coordinates
AB_havok = (U_tilde*S_tilde)*AB_tilde*pinv(U_tilde*S_tilde);

% System matrixes from HAVOK
A_havok = AB_havok(1:q*ny, 1:q*ny);
B_havok = AB_havok(1:q*ny, q*ny+1:end);
% A_havok = stabilise(A_havok,10);

% Make matrix sparse
A_havok(ny+1:end, :) = [eye((q-1)*ny), zeros((q-1)*ny, ny)]; % Add Identity matrix to carry delays over to x(k+1)
B_havok(ny+1:end, :) = zeros((q-1)*ny, nu); % Input has no effect on delays

%% Run with HAVOK (A_havok, B_havok and x)
% figure;
% plot(U1(:,1:5))
% title('First 5 modes of SVD')

% Compare to testing data
% Initial condition (last entries of training data)
y_hat_0 = zeros(q*ny,1); % Y[k] at top
for row = 0:q-1 % First column of spaced Hankel matrix
    y_hat_0(row*ny+1:(row+1)*ny, 1) = y_test(:,q-row);
end

% Run model
Y_hat = zeros(length(y_hat_0),N_test); % Empty estimated Y
Y_hat(:,q) = y_hat_0; % Initial condition
for k = q:N_test-1
    Y_hat(:,k+1) = A_havok*Y_hat(:,k) + B_havok*u_test(:,k);
end

y_hat_bar = Y_hat(1:ny, :); % Extract only non-delay time series

% Vector of Mean Absolute Error on testing data
MAE = sum(abs(y_hat_bar - y_test), 2)./N_test % For each measured state

%% Plot data vs model
figure;
plot(t_train, y_train);
hold on;
plot(t_test, y_test);

% plot(t_test, y_hat, 'k--', 'LineWidth', 1); % Plot only non-delay coordinate
plot(t_test, y_hat_bar, 'r--', 'LineWidth', 1); % Plot only non-delay coordinate
title('Training and Testing data vs Model (red = HAVOK, black = DMD)');
% legend('','','','','','', 'x hat','z hat','theta hat', 'x hat bar','z hat bar','theta hat bar');
hold off;

function A = stabilise(A_unstable,max_iterations)
    % If some eigenvalues are unstable due to machine tolerance,
    % Scale them to be stable
    A = A_unstable;
    count = 0;
    while (sum(abs(eig(A)) > 1) ~= 0)       
        [Ve,De] = eig(A);
        unstable = abs(De)>1; % indexes of unstable eigenvalues
        De(unstable) = De(unstable)./abs(De(unstable)) - 10^(-14 + count*2); % Normalize all unstable eigenvalues (set abs(eig) = 1)
        A = Ve*De/(Ve); % New A with margininally stable eigenvalues
        A = real(A);
        count = count+1;
        if(count > max_iterations)
            break
        end
    end

end