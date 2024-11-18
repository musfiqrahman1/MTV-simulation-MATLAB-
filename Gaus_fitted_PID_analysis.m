% MATLAB Code for PID Control Simulation with Calculation Menu

% Define Gaussian curve equation (thrust as a function of chamber pressure)
gaussian_curve = @(x) ...
    35.2635 * exp(-((x + 6.5205) / 1.7439)^2) + ...
    47.8407 * exp(-((x + 6.9833) / 0.1902)^2) + ...
    559.6959 * exp(-((x + 6.5290) / 0.0020)^2) + ...
    15.3750 * exp(-((x + 4.9538) / 0.0056)^2) + ...
    8.1547 * exp(-((x + 1.3883) / 0.6201)^2) + ...
    0.4184 * exp(-((x - 0.1653) / 0.0251)^2) + ...
    14.5778 * exp(-((x + 3.2504) / 1.5610)^2);

% Simulation parameters
time_step = 0.1;    % Simulation time step in seconds
total_time = 50;    % Total simulation time in seconds
sim_steps = total_time / time_step;

% PID controller parameters
Kp = 2;             % Proportional gain
Ki = 001;           % Integral gain
Kd = 0.01;           % Derivative gain

% Initialize variables
target_thrust = 50;  % Target thrust in Newtons
current_thrust = 0;   % Initial thrust
current_pressure = 0; % Initial chamber pressure
target_pressure = 0;  % To be calculated
error_integral = 0;   % Integral of error
previous_error = 0;   % Previous error for derivative calculation

% Find the chamber pressure corresponding to the target thrust
pressure_range = -10:0.01:10; % Define a range for chamber pressure
thrust_values = arrayfun(gaussian_curve, pressure_range); % Compute thrust for each pressure
[~, idx] = min(abs(thrust_values - target_thrust));       % Find the closest thrust value
target_pressure = pressure_range(idx);                   % Target chamber pressure

% Display initial target values
fprintf('=== Initial Calculations ===\n');
fprintf('Target Thrust: %.2f N\n', target_thrust);
fprintf('Target Chamber Pressure: %.2f\n\n', target_pressure);

% Simulation loop
time = 0:time_step:total_time;
pressure_values = zeros(size(time)); % To store pressure over time
thrust_values_sim = zeros(size(time)); % To store thrust over time

for i = 1:length(time)
    % Calculate the error
    error = target_pressure - current_pressure;

    % PID control calculations
    error_integral = error_integral + error * time_step;               % Integral term
    error_derivative = (error - previous_error) / time_step;           % Derivative term
    control_signal = Kp * error + Ki * error_integral + Kd * error_derivative; % PID formula

    % Update current pressure based on control signal (system dynamics)
    current_pressure = current_pressure + control_signal * time_step;

    % Calculate current thrust using the Gaussian curve
    current_thrust = gaussian_curve(current_pressure);

    % Store results for plotting
    pressure_values(i) = current_pressure;
    thrust_values_sim(i) = current_thrust;

    % Display calculations in command window
    fprintf('Time: %.1f s\n', time(i));
    fprintf('Target Thrust: %.2f N\n', target_thrust);
    fprintf('Target Chamber Pressure: %.2f\n', target_pressure);
    fprintf('Current Chamber Pressure: %.2f\n', current_pressure);
    fprintf('Current Thrust: %.2f N\n', current_thrust);
    fprintf('--------------------------------\n');

    % Update previous error
    previous_error = error;
end

% Plot the results
figure;
subplot(2, 1, 1);
plot(time, pressure_values, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Chamber Pressure');
title('Chamber Pressure vs Time');
grid on;

subplot(2, 1, 2);
plot(time, thrust_values_sim, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Thrust (N)');
title('Thrust vs Time');
grid on;

disp(['Target Pressure: ', num2str(target_pressure)]);
