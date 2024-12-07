% Linearized state-space matrices (replace with actual variable names from the project)
A = linsys.A; % State matrix
B = linsys.B; % Input matrix
C = linsys.C; % Output matrix
D = linsys.D; % Feedthrough matrix

% Índices relevantes
input_indices = [1, 2, 3, 4]; % Inputs dos 4 atuadores
output_index = 13; % Output do roll ('Euler(1)')

% Create the state-space system
sys = ss(A, B, C, D);

% Extrair matrizes do sistema original
[A, B, C, D] = ssdata(sys); % Obter as matrizes A, B, C e D

% Combinar os inputs dos atuadores em uma única entrada para rotação em roll
% Roll is influenced by differential thrust, e.g., (+T1 - T2 - T3 + T4)
roll_input_combination = [+1, -1, -1, +1]; % Combination pattern for roll
B_combined = B(:, input_indices) * roll_input_combination'; % Combine actuator effects
D_combined = D(output_index, input_indices) * roll_input_combination'; % Combine feedthrough effects

% Apply scaling to address small B matrix magnitude
input_scale = (1 / norm(B_combined)) * 1e5; % Scale factor for B matrix
output_scale = 1; % No scaling needed for roll output

% Apply scaling to system matrices
B_scaled = B_combined * input_scale; % Scale the input matrix
D_scaled = D_combined * input_scale; % Scale feedthrough term

% Create the scaled state-space system
roll_sys_scaled = ss(A, B_scaled, C(output_index, :), D_scaled);

% Criar novo sistema com entrada de diferencial de empuxo
roll_sys = ss(A, B_combined, C(output_index, :), D_combined);

% PID tuning
opts = pidtuneOptions('CrossoverFrequency', 0.4, 'PhaseMargin', 60);
[rollController, info] = pidtune(roll_sys_scaled, 'PD', opts);

% Extract PID gains
Kp = rollController.Kp;
Ki = rollController.Ki;
Kd = rollController.Kd / 10; % Optional scaling for Kd

% Display PID parameters
disp('PID Gains for Roll:');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);

% Validar o sistema em malha fechada
cl_roll_sys = feedback(rollController * roll_sys_scaled, 1);

% Resposta ao degrau para o sistema em malha fechada
figure;
step(cl_roll_sys);
title('Resposta ao Degrau - Malha Fechada (Roll)');
xlabel('Tempo (s)');
ylabel('Roll Angle (rad)');
