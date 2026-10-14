
%% DAY 4: UAV ALTITUDE CONTROL - DOUBLE INTEGRATOR
% Topic:
% 1. Transfer function
% 2. Poles and zeros
% 3. Step response
% 4. Impulse response
% 5. Delayed step input
% 6. Comparison of theoretical and simulated results
% 7. Iterative calculation of altitude

clc;        % Clear the Command Window
clear;      % Remove previously stored variables from memory
close all;  % Close all previously opened figure windows


%% SECTION 1: DEFINE THE SYSTEM

% Mass of the UAV in kilograms
m = 2;

% Force increment applied to the UAV in newtons
% This represents the change in thrust after accounting for gravity
U = 0.4;

% Define the transfer function:
%
% G(s) = Z(s)/U(s) = 1/(m*s^2)
%
% Here:
% Z(s) = Laplace transform of altitude
% U(s) = Laplace transform of the force increment
% m    = mass of the UAV
%
% The denominator [m 0 0] represents:
% m*s^2 + 0*s + 0
%
% Therefore, G(s) = 1/(m*s^2).

G = tf(1, [m 0 0]);

% Display the transfer function in the Command Window
disp('Transfer Function of the UAV:');
G


%% SECTION 2: FIND THE POLES AND ZEROS

% Poles are the roots of the denominator of the transfer function.
% For G(s) = 1/(m*s^2), the denominator is m*s^2.
% Both poles are located at s = 0.

p = pole(G);

disp('Poles of the system:');
disp(p);

% Zeros are the roots of the numerator.
% The numerator is the constant 1, so there are no finite zeros.

z = zero(G);

disp('Zeros of the system:');
disp(z);


%% SECTION 3: STEP RESPONSE

% A step input represents a force increment that suddenly changes
% from zero to a constant value and stays there.
%
% For this system:
% Acceleration = U/m
% Velocity     = (U/m)*t
% Altitude     = (U/(2*m))*t^2
%
% MATLAB's step(G) applies a UNIT step input.
% Its input magnitude is 1, not the U = 0.4 defined above.
%
% The following command plots the response for a unit step
% over 6 seconds.

figure;

step(G, 6);

grid on;
title('Unit Step Response of UAV Altitude');
xlabel('Time (seconds)');
ylabel('Altitude (meters)');


%% SECTION 4: IMPULSE RESPONSE

% An impulse represents a very short input with a finite total impulse.
% For an ideal double integrator, the altitude response to a unit
% impulse is proportional to time:
%
% z(t) = t/m
%
% This response differs from the quadratic step response.

figure;

impulse(G, 6);

grid on;
title('Unit Impulse Response of UAV Altitude');
xlabel('Time (seconds)');
ylabel('Altitude (meters)');


%% SECTION 5: CREATE A DELAYED STEP INPUT

% We now create our own input rather than using MATLAB's built-in step.
%
% The input will:
% 1. Remain zero before t = 1 second.
% 2. Jump to U = 0.4 N at t = 1 second.
% 3. Remain at 0.4 N afterward.

t = 0:0.01:6;

% Explanation of the time vector:
% Start at 0 seconds.
% Increase time by 0.01 seconds in each iteration.
% Stop at 6 seconds.
%
% This produces 601 time samples.

t0 = 1;

% Create a logical condition:
% t >= t0 is false before 1 second and true from 1 second onward.
%
% MATLAB converts false to 0 and true to 1.
% Multiplying by U produces the delayed step input.

u = U * (t >= t0);

% Display the delayed input so we can see when the force is applied.

figure;

plot(t, u, 'LineWidth', 1.5);

grid on;
title('Delayed Step Force Input');
xlabel('Time (seconds)');
ylabel('Force Increment (newtons)');


%% SECTION 6: SIMULATE THE SYSTEM USING LSIM

% lsim simulates a system's response to a user-defined input.
%
% G = the transfer function of the system
% u = the input force at each time sample
% t = the corresponding time values
%
% The output z contains the simulated altitude at each time sample.

z_sim = lsim(G, u, t);

% Plot the simulated altitude response.

figure;

plot(t, z_sim, 'b', 'LineWidth', 1.5);

grid on;
title('Simulated Altitude Response to Delayed Step');
xlabel('Time (seconds)');
ylabel('Altitude (meters)');


%% SECTION 7: CALCULATE THE THEORETICAL ALTITUDE

% From the equation of motion:
%
% m*d^2(z)/dt^2 = U
%
% Therefore:
%
% d^2(z)/dt^2 = U/m
%
% Assuming zero initial altitude and velocity, and a step applied
% at time t0, the theoretical altitude is:
%
% z(t) = U/(2*m) * (t - t0)^2, for t >= t0
% z(t) = 0,                       for t < t0
%
% max(t-t0, 0) ensures that the expression is zero before the step.

z_theory = (U/(2*m)) * max(t-t0, 0).^2;

% Important MATLAB operators:
% .^2 squares EACH element of the vector separately.
% Without the dot, ^ would attempt matrix exponentiation.


%% SECTION 8: COMPARE SIMULATED AND THEORETICAL RESULTS

% Plot both responses on the same graph.
% If the model and calculation agree, the curves should overlap
% apart from small numerical simulation errors.

figure;

plot(t, z_sim, 'b', 'LineWidth', 1.5);
hold on;  % Keep the first curve while adding the second

plot(t, z_theory, 'r--', 'LineWidth', 1.5);

grid on;

legend('MATLAB Simulation', 'Theoretical Equation', ...
       'Location', 'northwest');

title('Simulation vs Theoretical Altitude');
xlabel('Time (seconds)');
ylabel('Altitude (meters)');

hold off; % Finish adding curves to this figure


%% SECTION 9: ITERATE THROUGH SELECTED TIME VALUES

% A for loop repeats the same calculation for different time values.
% Here, we calculate the theoretical altitude at:
% 1, 2, 3, 4, 5 and 6 seconds.
%
% Each iteration:
% 1. Selects a time.
% 2. Calculates the elapsed time after the input starts.
% 3. Calculates the altitude using the theoretical equation.
% 4. Prints the result.

disp('Theoretical altitude at selected times:');

for time = 1:6

    % Calculate the time elapsed since the step began.
    % Before the step starts, elapsed time is zero.

    elapsed_time = max(time - t0, 0);

    % Calculate altitude at the current time.
    altitude = (U/(2*m)) * elapsed_time^2;

    % Print the result to three decimal places.
    fprintf('Time = %d s, Altitude = %.3f m\n', ...
            time, altitude);

end


%% SECTION 10: ITERATE THROUGH EVERY TIME SAMPLE

% This loop performs the theoretical calculation at every time sample.
% It demonstrates how the vectorized formula in Section 7 works.
%
% This is useful for understanding iterations, although the vectorized
% calculation is shorter and generally preferred in MATLAB.

z_loop = zeros(size(t));

% zeros(size(t)) creates an array of zeros with the same size as t.
% Each iteration fills one position in this array.

for k = 1:length(t)

    % Select the current time sample.
    current_time = t(k);

    % Determine how long the input has been active.
    elapsed_time = max(current_time - t0, 0);

    % Calculate altitude for this particular time sample.
    z_loop(k) = (U/(2*m)) * elapsed_time^2;

end

% Compare the loop result with the vectorized theoretical result.
% The difference should be approximately zero.

maximum_error = max(abs(z_loop - z_theory));

disp('Maximum difference between loop and vectorized results:');
disp(maximum_error);


%% END OF DAY 4 CODE
% You have now:
% 1. Defined a double-integrator transfer function.
% 2. Found its poles and zeros.
% 3. Plotted step and impulse responses.
% 4. Created a delayed step input.
% 5. Simulated the response using lsim.
% 6. Derived and plotted the theoretical response.
% 7. Used for loops to calculate altitude repeatedly.
% 8. Compared simulation results against the theoretical equation.
