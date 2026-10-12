%% Simplified UAV Vertical Motion (All 3 Cases)

% 1. Simulation Setup
dt = 0.01;
t = (0:dt:4)';      % Time vector (column vector)
u=0;
z0=0;

m = 2;              % UAV Mass (kg)
g = 9.81;           % Gravity (m/s^2)
T0 = m * g;         % Hover thrust (19.62 N)

% 2. The Three Thrust Cases
% Case 1: Under-thrust (T < mg) -> Drone sinks
% Case 2: Hover thrust (T = mg)  -> Drone stays still
% Case 3: Over-thrust (T > mg)   -> Drone rises
T_values = [T0 - 0.4, T0, T0 + 0.4];
labels = {'T < mg (Sinking)', 'T = mg (Hovering)', 'T > mg (Rising)'};

% 3. Calculate Motion for Each Case
a = zeros(length(t), 3);
v = zeros(length(t), 3);
z = zeros(length(t), 3);

for k = 1:3
    a(:,k) = (T_values(k) - m * g) / m;  % Acceleration
    v(:,k) =u + a(:,k) .* t;                % Velocity over time
    z(:,k) = z0+u.*t+0.5 * a(:,k) .* t.^2;       % Altitude over time
end

% 4. Plot the Results
figure;

subplot(3,1,1);
plot(t, a, 'LineWidth', 1.5);
title('Vertical Acceleration');
ylabel('a (m/s^2)');
grid on;
legend(labels, 'Location', 'best');

subplot(3,1,2);
plot(t, v, 'LineWidth', 1.5);
title('Vertical Velocity');
ylabel('v (m/s)');
grid on;

subplot(3,1,3);
plot(t, z, 'LineWidth', 1.5);
title('Altitude');
xlabel('Time (s)');
ylabel('z (m)');
grid on;