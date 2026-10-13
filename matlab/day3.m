%% Day 3: Operating points and numerical integration
clear; clc; close all;

%% Parameters
m  = 2;                  % Mass (kg)
g  = 9.81;               % Gravity (m/s^2)
T0 = m*g;                % Hover thrust (N)
U  = 0.4;                % Thrust increment (N)
x0 = [0; 0];             % Initial state [altitude dev (m); velocity (m/s)]
t  = (0:0.01:4)';        % Output times (s)

%% Define the two ODE models (state: x(1) = altitude, x(2) = velocity)
f_delta = @(t,x) [x(2); U/m];                 % Deviation model
f_total = @(t,x) [x(2); ((T0+U) - m*g)/m];    % Total-thrust model

%% Solve the ODEs
opts = odeset('RelTol',1e-9,'AbsTol',1e-11);
[~, x_delta] = ode45(f_delta, t, x0, opts);
[~, x_total] = ode45(f_total, t, x0, opts);

%% Closed-form reference
a = U/m;                              % Constant acceleration (m/s^2)
v_exact = x0(2) + a*t;
z_exact = x0(1) + x0(2)*t + 0.5*a*t.^2;

%% Errors
err_z      = max(abs(x_delta(:,1) - z_exact));
err_v      = max(abs(x_delta(:,2) - v_exact));
err_models = max(abs(x_delta(:) - x_total(:)));

%% Display results
fprintf('Hover thrust: %.3f N\n', T0);
fprintf('Total thrust: %.3f N\n', T0 + U);
fprintf('Acceleration: %.3f m/s^2\n', a);
fprintf('Final altitude: %.3f m\n', z_exact(end));
fprintf('Final velocity: %.3f m/s\n', v_exact(end));
fprintf('Maximum altitude error: %.3g m\n', err_z);
fprintf('Maximum velocity error: %.3g m/s\n', err_v);
fprintf('Maximum model difference: %.3g\n', err_models);

%% Plots
labels = {'Closed-form','Deviation ODE','Total-thrust ODE'};

figure;
plot(t, z_exact, 'k-', t, x_delta(:,1), 'b--', t, x_total(:,1), 'r:', 'LineWidth', 1.4);
grid on; xlabel('Time (s)'); ylabel('Altitude deviation (m)');
title('Day 3: Altitude Model Verification');
legend(labels, 'Location', 'best');

figure;
plot(t, v_exact, 'k-', t, x_delta(:,2), 'b--', t, x_total(:,2), 'r:', 'LineWidth', 1.4);
grid on; xlabel('Time (s)'); ylabel('Vertical velocity (m/s)');
title('Day 3: Velocity Model Verification');
legend(labels, 'Location', 'best');