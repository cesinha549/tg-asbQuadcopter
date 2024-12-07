% Linearized state-space matrices (replace with actual variable names from the project)
A = linsys.a; % State matrix
B = linsys.B; % Input matrix
C = linsys.C; % Output matrix
D = linsys.D; % Feedthrough matrix

% Índices relevantes
input_indices = [1, 2, 3, 4]; % Inputs dos 4 atuadores
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

% Criar novo sistema com empuxo total como única entrada
altitude_sys = ss(A, B_scaled, C(output_index, :), D_combined);

% Definir intervalos para PhaseMargin e CrossoverFrequency
phase_margin_values = 1:1:90;  % Intervalo para margem de fase (graus)
crossover_frequency_values = 0.1:0.1:15; % Intervalo para frequência de cruzamento (rad/s)

% Inicializar variáveis para armazenar os melhores resultados
best_Kp = 0;
best_Kd = 0;
best_Ki = 0;
best_phase_margin = 0;
best_crossover_frequency = 0;
min_overshoot = 5;

% Loop para testar diferentes combinações de PhaseMargin e CrossoverFrequency
for phase_margin = phase_margin_values
    for crossover_frequency = crossover_frequency_values
        try
            % Configurar as opções do PID Tuner
            opts = pidtuneOptions('CrossoverFrequency', crossover_frequency, 'PhaseMargin', phase_margin);
            
            % Calcular os ganhos PID
            [altitudeController, ~] = pidtune(altitude_sys, 'PD', opts);
            
            % Configurar o sistema em malha fechada
            cl_altitude_sys = feedback(altitudeController * altitude_sys, 1);
            
            % Calcular o step info
            step_info = stepinfo(cl_altitude_sys);
            
            % Verificar o overshoot
            if step_info.Overshoot >= min_overshoot && step_info.Overshoot <14 && step_info.SettlingTime < 1.2
                min_overshoot = step_info.Overshoot;
                best_Kp = altitudeController.Kp;
                best_Kd = altitudeController.Kd;
                best_Ki= altitudeController.Ki;
                best_phase_margin = phase_margin;
                best_crossover_frequency = crossover_frequency;
            end
        catch ME
            % Ignorar erros (como sistemas instáveis em algumas combinações)
            fprintf('Erro para PhaseMargin = %.2f, CrossoverFrequency = %.2f: %s\n', phase_margin, crossover_frequency, ME.message);
        end
    end
end

% Exibir os melhores resultados encontrados
fprintf('Melhores parâmetros encontrados:\n');
fprintf('PhaseMargin = %.2f graus\n', best_phase_margin);
fprintf('CrossoverFrequency = %.2f rad/s\n', best_crossover_frequency);
fprintf('Kp = %.4f\n', best_Kp);
fprintf('Kd = %.4f\n', best_Kd);
fprintf('Ki = %.4f\n', best_Ki);
fprintf('Menor Overshoot = %.4f%%\n', min_overshoot);

% Configurar o controlador com os melhores valores
opts = pidtuneOptions('CrossoverFrequency', best_crossover_frequency, 'PhaseMargin', best_phase_margin);
[altitudeController, ~] = pidtune(altitude_sys, 'PD', opts);

% Validar o sistema em malha fechada
cl_altitude_sys = feedback(altitudeController * altitude_sys, 1);

% Resposta ao degrau para o sistema em malha fechada
  [y, t] = step(cl_altitude_sys);
  steady_state_error = abs(1 - y(end));
figure;
step(cl_altitude_sys);
title('Resposta ao Degrau - Melhor Combinação de Overshoot (Altitude)');
xlabel('Tempo (s)');
ylabel('Altitude (m)');

% Mostrar stepinfo do melhor sistema
best_step_info = stepinfo(cl_altitude_sys);
disp('Step Info do Melhor Sistema:');
disp(best_step_info);
