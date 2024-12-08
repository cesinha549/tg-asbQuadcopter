
% Linearized state-space matrices (replace with actual variable names from the project)
A = linsys.a; % State matrix
B = linsys.B; % Input matrix
C = linsys.C; % Output matrix
D = linsys.D; % Feedthrough matrix


% Índices relevantes
input_indices = [1, 2, 3, 4]; % Inputs dos 4 atuadores
input_D = [1, 2]; % Inputs dos 4 atuadores
output_index = 18; % Output da altitude ('LLA(3)')


% Create the state-space system
sys = ss(A, B, C, D);

% Extrair matrizes do sistema original
[A, B, C, D] = ssdata(sys); % Obter as matrizes A, B, C e D

% Combinar os inputs dos atuadores em uma única entrada de empuxo total
B_combined = sum(B(:, input_indices), 2); % Soma das colunas correspondentes aos inputs relevantes
D_combined = sum(D(output_index, input_indices), 2); % Soma direta para o feedthrough

% Apply scaling to address small B matrix magnitude
input_scale = 1e14; % Scale factor for B matrix
output_scale = 1; % No scaling needed for output (C matrix is already well-scaled)

% Apply scaling to system matrices
B_scaled = B_combined * input_scale; % Scale the input matrix
D_scaled = D_combined * input_scale; % Scale feedthrough term

% Create the scaled state-space system
%altitude_sys_scaled = ss(A, B_scaled, C(output_index, :), D_scaled);

% Criar novo sistema com empuxo total como única entrada
altitude_sys = ss(A, B_scaled, C(output_index, :),D_combined);
%altitude_sys.InputDelay = 0.0001; % Add a delay of 0.01 seconds to the input

% PID tuning
opts = pidtuneOptions('CrossoverFrequency',10.7, 'PhaseMargin', 80);
[altitudeController, info] = pidtune(altitude_sys, 'PID', opts);

% Extract PID gains
Kp = altitudeController.Kp;
Ki = altitudeController.Ki;
Kd = altitudeController.Kd;

% Display PID parameters
disp('PID Gains for Altitude:');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);

% Validar o sistema em malha fechada
cl_altitude_sys = feedback(altitudeController * altitude_sys, 1);

 [y, t] = step(cl_altitude_sys);
 steady_state_error = abs(1 - y(end));

% Resposta ao degrau para o sistema em malha fechada
figure;
step(cl_altitude_sys);
stepinfo(cl_altitude_sys)
title('Resposta ao Degrau - Malha Fechada (Altitude)');
xlabel('Tempo (s)');
ylabel('Altitude (m)');


