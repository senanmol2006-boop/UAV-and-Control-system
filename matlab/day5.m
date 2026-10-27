
%% DAY 5: STATE-SPACE MODELLING
% Example: Vertical motion of a UAV
% Model: m*z_ddot = u
%
% States:
% x1 = z = altitude (m)
% x2 = v = vertical velocity (m/s)
%
% Input:
% u = thrust increment relative to hover (N)

clc;
clear;
close all;

%% 1. Define system parameters

m = 2;              % UAV mass (kg)
U = 0.4;             % Thrust increment (N)
t_step = 1;          % Input starts at 1 second

z0 = 0;              % Initial altitude (m)
v0 = 0;              % Initial velocity (m/s)

%% 2. Define state-space matrices

% State equation:
% dx/dt = A*x + B*u

A = [0 1;
     0 0];

B = [0;
     1/m];

% Output equation:
% y = C*x + D*u
%
% We want altitude as the output.

C = [1 0];
D = 0;

% Initial state vector:
% x = [altitude; velocity]

x0 = [z0; v0];

%% 3. Create the state-space system

sys = ss(A, B, C, D);

disp('State-Space Model:');
sys

%% 4. Define simulation time and input

t = (0:0.01:6)';

% Delayed step input:
% u = 0 before t_step
% u = U after t_step

u = U * (t >= t_step);

%% 5. Simulate the system

% lsim simulates the response to an arbitrary input.
% x0 specifies the initial state.

[y, t_out, x] = lsim(sys, u, t, x0);

% Output y is altitude because C = [1 0].
altitude = y;

% The second state is vertical velocity.
velocity = x(:,2);

%% 6. Calculate the analytical reference

% Expected altitude for the delayed step input.

altitude_ref = U * max(t_out - t_step, 0).^2 / (2*m);

% Expected velocity.

velocity_ref = (U/m) * max(t_out - t_step, 0);

%% 7. Plot altitude

figure;

plot(t_out, altitude, 'b', 'LineWidth', 2);
hold on;

plot(t_out, altitude_ref, 'r--', 'LineWidth', 1.5);

grid on;

xlabel('Time (s)');
ylabel('Altitude (m)');

title('UAV Altitude: State-Space Model');

legend('State-Space Simulation', ...
       'Analytical Reference', ...
       'Location', 'northwest');

%% 8. Plot velocity

figure;

plot(t_out, velocity, 'b', 'LineWidth', 2);
hold on;

plot(t_out, velocity_ref, 'r--', 'LineWidth', 1.5);

grid on;

xlabel('Time (s)');
ylabel('Velocity (m/s)');

title('UAV Vertical Velocity');

legend('State-Space Simulation', ...
       'Analytical Reference', ...
       'Location', 'northwest');

%% 9. Check maximum altitude error

error_altitude = altitude - altitude_ref;

max_error = max(abs(error_altitude));

fprintf('Maximum altitude error = %.10f m\n', max_error);

%% 10. Display selected results

check_times = [1 2 3 6];

fprintf('\nTime (s)    Altitude (m)    Velocity (m/s)\n');
fprintf('------------------------------------------\n');

for k = 1:length(check_times)

    [~, idx] = min(abs(t_out - check_times(k)));

    fprintf('%8.2f    %12.4f    %14.4f\n', ...
        t_out(idx), altitude(idx), velocity(idx));

end