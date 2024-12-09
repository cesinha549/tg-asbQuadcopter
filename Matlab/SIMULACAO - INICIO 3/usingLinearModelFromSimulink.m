% Name of the Simulink model and the linear model block
modelName = 'asbQuadcopter'; % Replace with your model's name
blockPath = 'linearAirframe/Linear/Linear Model'; % Path to the linear model block

% Load the Simulink model (if not already loaded)
load_system(modelName);

% Get the linearized system directly from the Simulink block
%linsys = linearize(modelName, blockPath);

% Display system information for debugging
disp('Linearized system:');
disp(linsys);

% Load the scope data (e.g., saved from Simulink)
load('custom_mux.mat'); % Replace with your actual file name

% Extract input signal and time from the loaded scope data
if exist('scope_mux', 'var')
    time = scope_mux.time; % Time vector
    input_signal = scope_mux.signals.values; % Input signal matrix
else
    error('The scope_mux.mat file does not contain the expected variables.');
end

sys = ss(linsys.a,linsys.b,linsys.c,linsys.d);

% Validate the input signal dimensions match the system inputs
% num_inputs = size(linsys.b, 2)% Number of inputs to the linearized system
% size(input_signal, 1)
% if size(input_signal, 2) ~= num_inputs
%     error('The number of columns in the input signal does not match the number of system inputs.');
% end

% Simulate the system's response to the input signal
[y, t] = lsim(sys, input_signal(:,:), time);

% Select the specific outputs to visualize (e.g., altitude or Euler angles)
% Adjust indices to match the desired output
output_indices = 30; % Example: output 8 (LLA, Altitude)
y_selected = y(:, output_indices);



% Plot the system's response
figure;
plot(t, y_selected);
grid on;
title('System Response to Scope Input Signal');
xlabel('Time (s)');
ylabel('System Output');
legend('System Output');



% Select a specific output (e.g., altitude)
output_index = output_indices; % Adjust based on your desired output
response = y(:, output_index); % Response for the selected output
steady_state_value = response(end); % Steady-state value

% Calculate Overshoot
overshoot = max(response) - steady_state_value;
overshoot_percentage = (overshoot / steady_state_value) * 100;

% Calculate Steady-State Error
reference_value = 1; % Replace with the desired reference
steady_state_error = abs(reference_value - steady_state_value);

% Calculate Settling Time
tolerance = 0.02; % 2% tolerance
settling_time = t(find(abs(response - steady_state_value) <= tolerance * steady_state_value, 1, 'last'));

% Display Results
fprintf('Overshoot: %.2f%%\n', overshoot_percentage);
fprintf('Steady-State Error: %.4f\n', steady_state_error);
fprintf('Settling Time: %.2f seconds\n', settling_time);