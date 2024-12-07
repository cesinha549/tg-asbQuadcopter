scenarios = {'Hover'};
TFinals = [25;30];

in = Simulink.SimulationInput.empty(size(scenarios, 1), 0);
for i = 1 : size(scenarios, 1)
    in(i) = Simulink.SimulationInput('asbQuadcopter');
    in(i) = in(i).setBlockParameter(['flightControlSystem/Flight Control System/' ...
        'landing logic/Position//Attitude Reference'], ...
        'ActiveScenario', scenarios{i});
    %% wind scenario
    if strcmp(scenarios{i}, 'WindZ')
        in(i) = in(i).setBlockParameter('nonlinearAirframe/Nonlinear/AC model/Applied Force Calculation/WindForce','ActiveScenario', scenarios{i});
    else
        in(i) = in(i).setBlockParameter('nonlinearAirframe/Nonlinear/AC model/Applied Force Calculation/WindForce','ActiveScenario', 'default');
    end
    in(i) = in(i).setVariable('TFinal', TFinals(i), 'Workspace', 'asbQuadcopter');
end

out = sim(in(1)); % simulate with the "Hover" scenario for 10 seconds


t = out.posref.time;
xyzrpy = out.xyzrpy;
estim = out.estim.signals.values;
posref = out.posref.signals.values;
motor = out.motor.signals.values;
sensor = out.sensor.signals.values;

scenarioName = scenarios{1}; % Nome do cenário atual (por exemplo, "Hover")

% Plot z-coordenada
figure();
plot(t, xyzrpy(:, 3), t, estim(:, 3), t, posref(:, 3), 'LineWidth', 2);
legend('Z Verdadeiro', 'Z Estimado', 'Z Referência', 'Location', 'best');
xlabel('Tempo [s]');
ylabel('z-coordenada [m]');
title([scenarioName '-z-coordenada']);
grid on;
saveas(gcf, [scenarioName '-z-coordenada.png']); % Salva o gráfico

% % Plot x-coordenada
% figure();
% plot(t, xyzrpy(:, 1) - xyzrpy(1, 1) * ones(size(t)), t, estim(:, 1), t, posref(:, 1), 'LineWidth', 2);
% legend('X Verdadeiro', 'X Estimado', 'X Referência', 'Location', 'best');
% xlabel('Tempo [s]');
% ylabel('x-coordenada [m]');
% title([scenarioName '-x-coordenada']);
% grid on;
% saveas(gcf, [scenarioName '-x-coordenada.png']); % Salva o gráfico
% 
% % Plot y-coordenada
% figure();
% plot(t, xyzrpy(:, 2) - xyzrpy(1, 2) * ones(size(t)), t, estim(:, 2), t, posref(:, 2), 'LineWidth', 2);
% legend('Y Verdadeiro', 'Y Estimado', 'Y Referência', 'Location', 'best');
% xlabel('Tempo [s]');
% ylabel('y-coordenada [m]');
% title([scenarioName '-y-coordenada']);
% grid on;
% saveas(gcf, [scenarioName '-y-coordenada.png']); % Salva o gráfico
% % 
% % Plot z-coordenada
% figure();
% plot(t, xyzrpy(:, 3), t, estim(:, 3), t, posref(:, 3), 'LineWidth', 2);
% legend('Z Verdadeiro', 'Z Estimado', 'Z Referência', 'Location', 'best');
% xlabel('Tempo [s]');
% ylabel('z-coordenada [m]');
% title([scenarioName '-z-coordenada']);
% grid on;
% saveas(gcf, [scenarioName '-z-coordenada.png']); % Salva o gráfico
% % 
% % Plot rolagem
% figure();
% plot(t, xyzrpy(:, 4), t, estim(:, 6), t, posref(:, 8), t, posref(:, 6), 'LineWidth', 2);
% legend('Rolagem Verdadeira ', 'Rolagem Estimada', 'Rolagem Comando', 'Rolagem Referência', 'Location', 'best');
% xlabel('Tempo [s]');
% ylabel('Rolagem [rad]');
% title([scenarioName '-rolagem']);
% grid on;
% saveas(gcf, [scenarioName '-rolagem.png']); % Salva o gráfico
% 
% % Plot arfagem
% figure();
% plot(t, xyzrpy(:, 5), t, estim(:, 5), t, posref(:, 7), t, posref(:, 5), 'LineWidth', 2);
% legend('Arfagem Verdadeira ', 'Arfagem Estimada', 'Arfagem Comando ', 'Arfagem Referência', 'Location', 'best');
% xlabel('Tempo [s]');
% ylabel('Arfagem [rad]');
% title([scenarioName '-arfagem']);
% grid on;
% saveas(gcf, [scenarioName '-arfagem.png']); % Salva o gráfico
% 
% % Plot guinada
% figure();
% plot(t, xyzrpy(:, 6), t, estim(:, 4), t, posref(:, 4), 'LineWidth', 2);
% legend('Guinada Verdadeira', 'Guinada Estimado', 'Guinada Referência ', 'Location', 'best');
% xlabel('Tempo [s]');
% ylabel('Guinada [rad]');
% title([scenarioName '-guinada']);
% grid on;
% saveas(gcf, [scenarioName '-guinada.png']); % Salva o gráfico
% 
% 
% Calcular overshoot para a z-coordenada
z_true = xyzrpy(:, 3); % Extraindo a z-coordenada verdadeira
z_ref = posref(:, 3); % Extraindo a referência
% 
% % % Encontrar o valor de regime (média dos últimos valores para evitar ruído)
z_final = mean(z_true(end-50:end)); % Considerando os últimos 50 pontos como regime
% % 
% % % Encontrar o valor de pico (máximo) antes do tempo de regime
% % [~, idx_peak] = min(z_true);
% % z_peak = z_true(idx_peak);
% % 
% % % Calcular o overshoot em percentual
% % overshoot_percentage = ((z_peak - z_final) / abs(z_final)) * 100;
% % 
% % % Exibir resultado
% % fprintf('O overshoot da z-coordenada é de aproximadamente %.2f%%\n', overshoot_percentage);
% % 
% % 
% % % Calcular tempor de assentamento para a z-coordenada
% % % Definindo o critério de tolerância (2% do valor final)
% % tolerance = 0.02;
% % 
% % % Extraindo a resposta verdadeira e o tempo
% % z_true = xyzrpy(:, 3); % Extraindo a z-coordenada verdadeira
% % t = out.posref.time; % Extraindo o vetor de tempo
% % 
% % % Encontrando o valor de regime (média dos últimos 50 pontos para reduzir ruído)
% % z_final = mean(z_true(end-50:end));
% % 
% % % Encontrando o tempo de assentamento
% % settling_time_index = find(abs(z_true - z_final) > tolerance * abs(z_final), 1, 'last');
% % settling_time = t(settling_time_index);
% % 
% % % Exibir resultado
% % fprintf('O tempo de assentamento para a z-coordenada é de aproximadamente %.2f segundos.\n', settling_time);
% % 
% % 
% % % Calcular erro em regime permanente para a z-coordenada
% % % Extraindo a saída verdadeira e a referência
% % z_true = xyzrpy(:, 3); % Extraindo a z-coordenada verdadeira
% % z_ref = posref(:, 3); % Extraindo a referência
% % 
% % % Encontrando o valor final da saída e da referência (média dos últimos 50 pontos)
% % z_final = mean(z_true(end-50:end)); % Saída em regime
% % z_ref_final = mean(z_ref(end-50:end)); % Referência em regime
% % 
% % % Calculando o erro em regime permanente
% % steady_state_error = z_ref_final - z_final;
% % 
% % % Exibir resultado
% % fprintf('O erro em regime permanente para a z-coordenada é de aproximadamente %.4f metros.\n', steady_state_error);
% % 
% % 
% % % % Definir tolerâncias para tempo de assentamento
% % % settling_tolerance = 0.02; % 2% do valor final
% % % 
% % % % Inicializar resultados
% % % results = struct('overshoot', [], 'settling_time', [], 'steady_state_error', []);
% % % 
% % % % Loop para as 6 variáveis de saída (x, y, z, roll, pitch, yaw)
% % % for i = 1:6
% % %     % Extrair a resposta e a referência
% % %     response = xyzrpy(:, i);
% % %     reference = posref(:, i);
% % %     t = out.posref.time; % Tempo de simulação
% % % 
% % %     % Encontrar o valor final da referência e da resposta (média dos últimos 50 pontos)
% % %     final_response = mean(response(end-50:end));
% % %     final_reference = mean(reference(end-50:end));
% % % 
% % %     % Verificar se o sistema é estável
% % %     is_stable = all(abs(response(end-50:end) - final_response) < settling_tolerance * abs(final_response));
% % % 
% % %     if is_stable
% % %         % Calcular o overshoot
% % %         if final_reference >= 0
% % %             [peak_value, ~] = max(response);
% % %         else
% % %             [peak_value, ~] = min(response); % Inverte para referências negativas
% % %         end
% % %         overshoot = ((peak_value - final_reference) / abs(final_reference)) * 100;
% % % 
% % %         % Calcular o tempo de assentamento
% % %         settling_time_index = find(abs(response - final_response) > settling_tolerance * abs(final_response), 1, 'last');
% % %         settling_time = t(settling_time_index);
% % % 
% % %         % Calcular o erro em regime permanente
% % %         steady_state_error = final_reference - final_response;
% % %     else
% % %         % Definir valores para sistema instável
% % %         overshoot = NaN;
% % %         settling_time = Inf;
% % %         steady_state_error = NaN;
% % %     end
% % % 
% % %     % Salvar os resultados
% % %     results(i).overshoot = overshoot;
% % %     results(i).settling_time = settling_time;
% % %     results(i).steady_state_error = steady_state_error;
% % % 
% % %     % Adicionar texto com os resultados no gráfico
% % %     figure(i);
% % %     plot(t, response, 'LineWidth', 2);
% % %     hold on;
% % %     plot(t, reference, '--', 'LineWidth', 2);
% % %     xlabel('Tempo [s]');
% % %     ylabel(sprintf('Coordenada %d [unidade]', i));
% % %     title(sprintf('Resposta do Grau de Liberdade %d', i));
% % %     grid on;
% % %     legend('Resposta', 'Referência');
% % % 
% % %     % Mostrar resultados no gráfico
% % %     annotation('textbox', [0.15, 0.75, 0.1, 0.1], 'String', sprintf('Overshoot: %.2f%%\nTempo de Assentamento: %.2f s\nErro em Regime Permanente: %.4f', overshoot, settling_time, steady_state_error), 'FitBoxToText', 'on');
% % % end
% % % 
% % % % Exibir os resultados
% % % for i = 1:6
% % %     fprintf('Grau de liberdade %d:', i);
% % %     fprintf('  Overshoot: %.2f%%', results(i).overshoot);
% % %     fprintf('  Tempo de assentamento: %.2f segundos', results(i).settling_time);
% % %     fprintf('  Erro em regime permanente: %.4f', results(i).steady_state_error);
% % %     fprintf('\n');
% % % end
% % 
% % % Extrair os dados da simulação
% % altitude_response = xyzrpy(:, 3); % Coordenada Z representa a altitude
% % altitude_reference = posref(:, 3); % Referência de altitude
% % t = out.posref.time; % Tempo de simulação
% % 
% % % Encontrar o valor final de resposta e de referência
% % final_response = mean(altitude_response(end-50:end));
% % final_reference = mean(altitude_reference(end-50:end));
% % 
% % % Encontrar o valor máximo (pico) da resposta
% % [peak_value, ~] = min(altitude_response);
% % 
% % % Calcular o overshoot em percentual
% % overshoot = ((peak_value - final_reference) / abs(final_reference)) * 100;
% % 
% % % Exibir resultado
% % fprintf('O overshoot para a mudança de altitude é de aproximadamente %.2f%%\n', overshoot);
