%% Define Scenarios and Final Times
scenarios = {'Hover'}; % Add more scenarios as needed
TFinals = [10, 30]; % Simulation times for each scenario

%% Initialize Simulation Input Objects
in = Simulink.SimulationInput.empty(size(scenarios, 2), 0);
for i = 1:length(scenarios)
    % Create simulation input for each scenario
    in(i) = Simulink.SimulationInput('asbQuadcopter');
    in(i) = in(i).setBlockParameter(['flightControlSystem/Flight Control System/' ...
        'landing logic/Position//Attitude Reference'], ...
        'ActiveScenario', scenarios{i});
    
    % Configure wind scenario if applicable
    if strcmp(scenarios{i}, 'WindZ')
        in(i) = in(i).setBlockParameter('nonlinearAirframe/Nonlinear/AC model/Applied Force Calculation/WindForce', ...
            'ActiveScenario', scenarios{i});
    else
        in(i) = in(i).setBlockParameter('nonlinearAirframe/Nonlinear/AC model/Applied Force Calculation/WindForce', ...
            'ActiveScenario', 'default');
    end
    
    % Set the final simulation time
    in(i) = in(i).setVariable('TFinal', TFinals(i), 'Workspace', 'asbQuadcopter');
end

% set_param('asbQuadcopter/Command/Signal Editor/Bus Creator', 'OutDataTypeStr', 'Bus: myBusType');
% 
% 
% % Set the solver to variable-step
% set_param('asbQuadcopter', 'Solver', 'VariableStepAuto');
% 
% % Disable 'Treat each discrete rate as a separate task'
% set_param('asbQuadcopter', 'EnableMultiTasking', 'off');

%% Define Linearization I/O Points
% Adjust block paths to match your model
io = [
    linio('flightController/Flight Controller/gravity feedforward/Sum3', 1, 'input'), ...  % Input to PID controller
    linio('flightController/Flight Controller/gravity feedforward/Sum4', 1, 'output')     % Altitude output
]



io2 = getlinio('flightController')

%% Simulation and Linearization
results = struct('Scenario', [], 'LinearModel', [], 'PID', [], 'Overshoot', [], 'SettlingTime', [], 'SteadyStateError', []);
for i = 1:length(scenarios)
    % Set active scenario
    scenarioName = scenarios{i};
    disp(['Running simulation for scenario: ', scenarioName]);
    
    % Simulate the model
    out = sim(in(i));
    
    % Linearize the model
    sys = linearize('flightController', io);
    opts = pidtuneOptions('DesignFocus', 'reference-tracking', 'PhaseMargin', 60);
    
    bode(sys);
    tf(sys)
    
    
%     opspec = operspec('asbQuadcopter');
%     op = findop('asbQuadcopter', opspec);
%     sys2 = linearize('asbQuadcopter', op, io);
    
    % Tune PID Controller
    C = pidtune(sys, 'PD',opts);
    fprintf('Scenario: %s\n', scenarioName);
    fprintf('Tuned PID Parameters:\nKp = %.4f, Ki = %.4f, Kd = %.4f\n', C.Kp, C.Ki, C.Kd);
    
    % Update model with tuned PID parameters
    %set_param('flightControlSystem/PID_Controller', 'P', num2str(C.Kp));
    %set_param('flightControlSystem/PID_Controller', 'I', num2str(C.Ki));
    %set_param('flightControlSystem/PID_Controller', 'D', num2str(C.Kd));
    
    % Analyze Performance Metrics
    t = out.posref.time; % Time vector
    z_response = out.xyzrpy(:, 3); % Altitude (z-axis)
    z_reference = out.posref.signals.values(:, 3); % Altitude reference
    
    % Calculate Overshoot
    z_final = mean(z_response(end-50:end)); % Final altitude
    [z_peak, ~] = max(z_response); % Peak altitude
    overshoot = ((z_peak - z_final) / abs(z_final)) * 100;
    
    % Calculate Settling Time
    tolerance = 0.02; % 2% tolerance
    settling_time_index = find(abs(z_response - z_final) > tolerance * abs(z_final), 1, 'last');
    settling_time = t(settling_time_index);
    
    % Calculate Steady-State Error
    steady_state_error = z_reference(end) - z_final;
    
    % Save Results
    results(i).Scenario = scenarioName;
    results(i).LinearModel = sys;
    results(i).PID = C;
    results(i).Overshoot = overshoot;
    results(i).SettlingTime = settling_time;
    results(i).SteadyStateError = steady_state_error;
end

%% Display Results
for i = 1:length(results)
    fprintf('Scenario: %s\n', results(i).Scenario);
    fprintf('  Overshoot: %.2f%%\n', results(i).Overshoot);
    fprintf('  Settling Time: %.2f seconds\n', results(i).SettlingTime);
    fprintf('  Steady-State Error: %.4f meters\n', results(i).SteadyStateError);
    fprintf('  PID Gains: Kp = %.4f, Ki = %.4f, Kd = %.4f\n\n', ...
        results(i).PID.Kp, results(i).PID.Ki, results(i).PID.Kd);
end
