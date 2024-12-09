% Name of the Simulink model and the linear model block
modelName = 'asbQuadcopter'; % Replace with your model's name
blockPath = 'linearAirframe/Linear/Linear Model'; % Path to the linear model block

% Load the Simulink model (if not already loaded)
load_system(modelName);

% Create a state-space system from the linearized system
sys = ss(linsys.a, linsys.b, linsys.c, linsys.d);

% Select the SISO input-output pair
altitude_input_index = [1,2,3,4]; % Replace with the input index controlling altitude (e.g., thrust)
altitude_output_index = 30; % Replace with the output index for altitude (e.g., LLA(3))
b_column_input = sys.B(:, altitude_input_index);
b_column_input = b_column_input(:);

a_column_input = sys.A(:, altitude_input_index);
a_column_input = a_column_input(:);
a_column_input = repmat(a_column_input, 1, 48);

c_input = sys.C(altitude_output_index, :);

d_input = sys.D(altitude_output_index, altitude_input_index);

% Replicar as colunas para alcançar 48 colunas
factor = ceil(48 / size(c_input, 2)); % Determina quantas vezes replicar as colunas
c_expanded = repmat(c_input, 1, factor); % Replica as colunas da matriz original

% Reduzir para 48 colunas
c_transformed = c_expanded(:, 1:48); % Seleciona apenas as primeiras 48 colunas

%%%%%%%%%%%

% Extract the SISO system
sys_altitude = ss(a_column_input, b_column_input, c_transformed, sys.D(altitude_output_index, 1));

% Check the dimensions of the SISO system
disp('SISO system for altitude:');
disp(sys_altitude);

% Tuning the PID controller
opts = pidtuneOptions('CrossoverFrequency', 1, 'PhaseMargin', 60); % Adjust tuning parameters
[altitude_pid, tuning_info] = pidtune(sys_altitude, 'PID', opts);

% Display the tuned PID gains
disp('Tuned PID Gains:');
disp(altitude_pid);

% Simulate the closed-loop response with the tuned PID controller
closed_loop_sys = feedback(altitude_pid * sys_altitude, 1);

% Step response for evaluation
figure;
step(closed_loop_sys);
grid on;
title('Closed-Loop Step Response (Altitude)');
xlabel('Time (s)');
ylabel('Altitude');

% Simulate the response to your input signal
load('custom_mux.mat'); % Replace with your file name
if exist('scope_mux', 'var')
    time = scope_mux.time; % Time vector
    input_signal = scope_mux.signals.values(:, :); % Input signal for altitude
else
    error('The scope_mux.mat file does not contain the expected variables.');
end

% Ensure the time vector matches the input signal
% % Define the start and end time, and the number of time steps
% start_time = 0; % Start of the time vector
% end_time = 1;   % End of the time vector
% num_steps = 14; % Number of time steps to match input_signal

% % Generate a linearly spaced time vector
% time = linspace(start_time, end_time, num_steps)'; % Column vector (14x1)
% altitude_input_signal = input_signal(:, :); % Use the specific input for altitude control
% Simulate the closed-loop system response to the input signal


altsig = altitude_input_signal';
% Suponha que altsig é a matriz original
[m, n] = size(altsig); % Obter o tamanho da matriz (m linhas, n colunas)

% Adicionar uma coluna de zeros
altsig = [altsig, zeros(m, 1)]; % Adiciona uma coluna de zeros ao final

disp(['Time vector length: ', num2str(length(time))]);
disp(['Input signal rows: ', num2str(size(altsig, 1))]);
disp(['Input signal columns: ', num2str(size(altsig, 2))]);
disp(['INPUTS CHANNELS: ', size(closed_loop_sys.b, 2)]);

[y, t] = lsim(closed_loop_sys, altsig(:, 1), time);

% Plot the system response
figure;
plot(t, y);
grid on;
title('Closed-Loop System Response to Input Signal (Altitude)');
xlabel('Time (s)');
ylabel('Altitude');

% Analyze performance metrics
steady_state_value = y(end); % Steady-state value
overshoot = max(y) - steady_state_value;
overshoot_percentage = (overshoot / steady_state_value) * 100;
settling_time = t(find(abs(y - steady_state_value) <= 0.02 * steady_state_value, 1, 'last'));
steady_state_error = abs(reference_value - steady_state_value); % Reference vs actual

% Display metrics
fprintf('Overshoot: %.2f%%\n', overshoot_percentage);
fprintf('Steady-State Error: %.4f\n', steady_state_error);
fprintf('Settling Time: %.2f seconds\n', settling_time);
