
clear; clc;

% Simulation parameters
desired_thrust = 100; % Desired thrust in N
time_step = 0.1; % Time interval in seconds
sim_time = 4; % Total simulation time in seconds
time = 0:time_step:sim_time; % Time vector


% PID controller parameters (tuned as needed)
Kp = 0.5; % Proportional gain
Ki = 0.001; % Integral gain
Kd = 0.02; % Derivative gain

% Initialize variables for PID control
integral = 0;
previous_error = 0;
current_thrust = 0; % Start from 0 as specified

% Arrays to store results for plotting
actual_thrust = zeros(1, length(time));
desired_thrust_values = desired_thrust * ones(1, length(time));

% PID Control Simulation Loop
for i = 1:length(time)
    % Error calculation
    error = desired_thrust - current_thrust;
    
    % PID calculations
    integral = integral + error * time_step;
    derivative = (error - previous_error) / time_step;
    control_output = Kp * error + Ki * integral + Kd * derivative;
    
    % Update thrust based on control output
    current_thrust = current_thrust + control_output;
    
    % Store results for plotting
    actual_thrust(i) = current_thrust;
    
    % Update for next iteration
    previous_error = error;
end

% Plot the results
figure;
plot(time, desired_thrust_values, 'r--', 'LineWidth', 1.5); % Desired thrust
hold on;
plot(time, actual_thrust, 'b-', 'LineWidth', 1.5); % Actual thrust output
xlabel('Time (s)');
ylabel('Thrust (N)');
title('PID Control for Thrust Stabilization');
legend('Desired Thrust', 'Actual Thrust Output');
grid on;
hold off;
