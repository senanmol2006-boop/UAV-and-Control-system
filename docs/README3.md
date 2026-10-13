# Day 3: Operating Points, Hover Deviations & Numerical Integration

Day 3 builds on the vertical-motion model from Day 2. It shows how to describe UAV motion relative to a hover operating point, derive a simplified deviation model, and solve the resulting differential equations numerically.

The day also introduces numerical integration with MATLAB's `ode45` solver and the forward Euler method. Numerical results are checked against analytical solutions, and the remaining differences are analysed in both MATLAB and Simulink.



## Learning Objectives

- Understand operating points, equilibrium, and initial conditions.
- Express total thrust as hover thrust plus a thrust increment.
- Derive the vertical-motion equation in deviation form.
- Understand why gravity cancels in the ideal hover-deviation model.
- Distinguish affine models, linear models, and linearization.
- Convert a second-order differential equation into a first-order ODE system.
- Implement the model using MATLAB's `ode45`.
- Understand forward Euler integration and its numerical error.
- Compare absolute and deviation models in Simulink.
- Distinguish numerical error, parameter error, and model error.
- Interpret floating-point round-off in MATLAB and Simulink results.

---

## 1. Review of the Vertical-Motion Model

Taking upward as the positive direction, the forces acting on the UAV are thrust $T$ and gravitational force $mg$.

Using Newton's second law:

$$m\ddot z = T - mg$$

Therefore, the vertical acceleration is:

$$\ddot z = \frac{T}{m} - g$$

At steady hover, thrust balances weight:

$$T_0 = mg$$

For a UAV with mass $m = 2\ \text{kg}$ and $g = 9.81\ \text{m/s}^2$:

$$T_0 = 2(9.81) = 19.62\ \text{N}$$

This hover thrust becomes the reference input for the Day 3 deviation model.

---

## 2. Operating Point vs. Initial Condition

An **operating point** is the nominal condition around which the system is studied. For steady hover, it consists of a constant altitude, zero vertical velocity, and thrust equal to weight.

An **initial condition** specifies the state at the beginning of a particular simulation.

For example:

$$z_{\text{eq}} = 10\ \text{m}, \qquad v_{\text{eq}} = 0$$

describes a hover operating point, whereas

$$z(0) = 10\ \text{m}, \qquad v(0) = 0.2\ \text{m/s}$$

describes a simulation that starts at the same altitude but with upward velocity.

With zero net force, the UAV continues moving upward at constant velocity in the ideal model. Therefore, zero acceleration does not necessarily mean the UAV is hovering.

| Concept | Meaning |
|---|---|
| Operating point | Nominal condition used as a reference |
| Initial condition | State at the start of a simulation |
| Equilibrium | State that remains constant under the corresponding input |

---

## 3. Deriving the Hover-Deviation Model

### 3.1 Define the thrust increment

Instead of using total thrust directly, define the input as a change from hover thrust:

$$T = T_0 + u$$

Where:

- $T$ is total thrust in newtons.
- $T_0$ is hover thrust in newtons.
- $u$ is the thrust increment in newtons.

For example, if $T_0 = 19.62\ \text{N}$ and $u = 0.40\ \text{N}$:

$$T = 19.62 + 0.40 = 20.02\ \text{N}$$

### 3.2 Substitute into Newton's second law

Starting from:

$$m\ddot z = T - mg$$

Substituting $T = T_0 + u$:

$$m\ddot z = T_0 + u - mg$$

At hover, $T_0 = mg$, so:

$$m\ddot z = mg + u - mg$$

The hover thrust and weight cancel:

$$m\ddot z = u$$

Therefore, the deviation equation is:

$$\ddot z = \frac{u}{m}$$

> **Interpretation:** Gravity has not disappeared physically. Its effect is balanced by the baseline hover thrust. The deviation equation describes motion caused by changes in thrust relative to that baseline.

### 3.3 Absolute vs. deviation models

| Model | Equation | Input |
|---|---|---|
| Absolute model | $\ddot z = (T - mg)/m$ | Total thrust $T$ |
| Deviation model | $\ddot z = u/m$ | Thrust increment $u$ |

Both models produce the same acceleration when:

$$T = mg + u$$

The input definitions must not be mixed.

---

## 4. Position Deviation and Altitude Datum

Position can also be expressed relative to the hover altitude:

$$z = z_{\text{eq}} + \delta z$$

For a constant hover altitude, the derivatives of $z_{\text{eq}}$ are zero. Therefore:

$$\delta\ddot z = \frac{u}{m}$$

If the UAV hovers at $10\ \text{m}$, its deviation altitude is zero at that operating point.

The physical altitude is reconstructed using:

$$z_{\text{physical}} = z_{\text{eq}} + \delta z$$

For example, a deviation altitude of $0.5\ \text{m}$ from a $10\ \text{m}$ hover point corresponds to a physical altitude of $10.5\ \text{m}$.

This distinction is important when a simulation starts at zero altitude deviation even though the UAV is already airborne.

---

## 5. Affine Models and Linearization

The absolute equation is:

$$\ddot z = \frac{T}{m} - g$$

It contains a linear term in thrust and a constant offset. This makes it an **affine** relationship.

After expressing thrust as a deviation from hover:

$$\ddot z = \frac{u}{m}$$

the constant gravity offset cancels, leaving a linear relationship between thrust increment and acceleration.

This cancellation is exact under the model assumptions; it does not require a small thrust increment.

### Rotor-speed linearization

For the illustrative quadratic rotor model:

$$T_i = k_T \Omega_i^2$$

A first-order Taylor approximation around rotor speed $\Omega_0$ gives:

$$\delta T_i \approx 2 k_T \Omega_0 \, \delta\Omega_i$$

For a 5% increase in rotor speed:

| Quantity | Result |
|---|---|
| Exact thrust increase | 10.25% of baseline |
| Linearized thrust increase | 10% of baseline |
| Difference | 0.25% of baseline |

The approximation becomes less accurate as the operating point moves farther from the expansion point.

For tilted thrust, the vertical component is:

$$T_z = T\cos\theta$$

The cosine relationship is nonlinear. Near zero tilt, the loss in vertical thrust begins at second order in the small tilt angle.

---

## 6. First-Order State-Space Representation

The deviation model is a second-order differential equation:

$$\ddot z = \frac{u}{m}$$

Define the state variables:

$$x_1 = z, \qquad x_2 = v$$

Then:

$$\dot x_1 = x_2$$

$$\dot x_2 = \frac{u}{m}$$


---

## 7. Analytical Solution

For a constant thrust increment $u$, acceleration is constant:

$$a = \frac{u}{m}$$

Integrating acceleration gives velocity:

$$v(t) = v_0 + \frac{u}{m}\,t$$

Integrating velocity gives altitude deviation:

$$z(t) = z_0 + v_0 t + \frac{u}{2m}\,t^2$$

These equations provide an analytical reference against which numerical results can be checked.

### Worked example

Given:

$$m = 2\ \text{kg}, \quad u = 0.4\ \text{N}$$

$$z_0 = 1\ \text{m}, \quad v_0 = 0.3\ \text{m/s}$$

At $t = 4\ \text{s}$:

$$v(4) = 0.3 + \frac{0.4}{2}(4) = 1.1\ \text{m/s}$$

The altitude is:

$$z(4) = 1 + 0.3(4) + \frac{0.4}{2(2)}(4^2) = 3.8\ \text{m}$$

---

## 8. Numerical Integration

Numerical integration approximates the solution of a differential equation by advancing the system through time.

### MATLAB `ode45`

`ode45` is an adaptive numerical solver for nonstiff ordinary differential equations. It adjusts internal step sizes to control numerical error.

The ODE function must return the derivatives of the state variables:

$$
\dot x =
\begin{bmatrix}
v \\
u/m
\end{bmatrix}
$$

The solver uses these derivatives and the initial state to calculate the trajectory. The initial state `x0` is only the first value passed to the function; every later state is generated by the solver.

Specifying an output time vector does not mean that every internal solver step has the same spacing.

### Forward Euler method

Forward Euler estimates the next state using the derivative at the beginning of the time step:

$$v_{n+1} = v_n + a_n h$$

$$z_{n+1} = z_n + v_n h$$

Where $h$ is the time-step size.

For constant acceleration, Euler gives the sampled velocity exactly in this example, but its altitude calculation underestimates the exact displacement from rest.

For $a = 0.2\ \text{m/s}^2$, $z_0 = v_0 = 0$, and four seconds of motion:

| Step size | Euler altitude | Exact altitude | Absolute error |
|---:|---:|---:|---:|
| 1.0 s | 1.2 m | 1.6 m | 0.4 m |
| 0.5 s | 1.4 m | 1.6 m | 0.2 m |

Reducing the step size improves the result in this example.

---

## 9. MATLAB Implementation

The following script compares the absolute model, deviation model, and analytical solution. Save it as `day03_ode.m`.

```matlab
%% Day 3: Operating Points and Numerical Integration
clear; clc; close all;

%% Parameters
m  = 2;                  % Mass (kg)
g  = 9.81;               % Gravity (m/s^2)
T0 = m*g;                % Hover thrust (N)
U  = 0.4;                % Thrust increment (N)

z0 = 0;                  % Initial altitude deviation (m)
v0 = 0;                  % Initial vertical velocity (m/s)

t  = (0:0.01:4)';        % Requested output times (s)
x0 = [z0; v0];           % Initial state [altitude; velocity]

%% ODE functions
% x(1) = altitude, x(2) = velocity
f_delta = @(t,x) [x(2); U/m];                  % Deviation model
f_total = @(t,x) [x(2); ((T0 + U) - m*g)/m];   % Total-thrust model

%% Numerical solutions
opts = odeset('RelTol',1e-9,'AbsTol',1e-11);

[~, x_delta] = ode45(f_delta, t, x0, opts);
[~, x_total] = ode45(f_total, t, x0, opts);

%% Analytical solution
v_exact = v0 + (U/m)*t;
z_exact = z0 + v0*t + (U/(2*m))*t.^2;

%% Error calculations
err_z      = max(abs(x_delta(:,1) - z_exact));   % ODE vs analytical (altitude)
err_v      = max(abs(x_delta(:,2) - v_exact));   % ODE vs analytical (velocity)
err_models = max(abs(x_delta(:) - x_total(:)));  % Deviation vs total-thrust model

%% Results
fprintf('Hover thrust: %.3f N\n', T0);
fprintf('Total thrust: %.3f N\n', T0 + U);
fprintf('Acceleration: %.3f m/s^2\n', U/m);
fprintf('Final altitude: %.3f m\n', z_exact(end));
fprintf('Final velocity: %.3f m/s\n', v_exact(end));

fprintf('Maximum altitude error: %.3g m\n', err_z);
fprintf('Maximum velocity error: %.3g m/s\n', err_v);
fprintf('Maximum model difference: %.3g\n', err_models);

%% Altitude plot
figure;
plot(t, z_exact, 'k-', ...
     t, x_delta(:,1), 'b--', ...
     t, x_total(:,1), 'r:', 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('Altitude deviation (m)');
title('Day 3: Altitude Model Verification');
legend('Analytical','Deviation ODE','Total-thrust ODE','Location','best');

%% Velocity plot
figure;
plot(t, v_exact, 'k-', ...
     t, x_delta(:,2), 'b--', ...
     t, x_total(:,2), 'r:', 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('Vertical velocity (m/s)');
title('Day 3: Velocity Model Verification');
legend('Analytical','Deviation ODE','Total-thrust ODE','Location','best');

%% Error plot (log scale)
% eps is added only so that exact zeros can be drawn on a log axis
figure;
semilogy(t, abs(x_delta(:,1) - z_exact)       + eps, 'b-',  ...
         t, abs(x_delta(:,2) - v_exact)       + eps, 'r-',  ...
         t, abs(x_delta(:,1) - x_total(:,1))  + eps, 'k--', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Absolute error');
title('Day 3: ODE Error vs. Time');
legend('Altitude: ODE vs analytical', ...
       'Velocity: ODE vs analytical', ...
       'Altitude: deviation vs total-thrust', 'Location','best');

%% Save results
if ~exist('results','dir')
    mkdir('results');
end

save('results/day03.mat', ...
     't','x_delta','x_total','z_exact','v_exact', ...
     'err_z','err_v','err_models','m','g','T0','U');
```

### Expected results

For the default parameters:

| Quantity | Value |
|---|---|
| Hover thrust | 19.62 N |
| Total thrust | 20.02 N |
| Acceleration | 0.20 m/s² |
| Final velocity at 4 s | 0.80 m/s |
| Final altitude deviation at 4 s | 1.60 m |

The numerical and analytical trajectories should agree within the specified numerical tolerances. See [Error Analysis](#11-error-analysis) for how to interpret the printed error values.

---

## 10. Simulink Implementation

The Day 2 absolute model can be extended with a parallel deviation-model branch.

### Blocks used

| Block | Library | Purpose |
|---|---|---|
| Constant | Sources | Total thrust $T$, weight $mg$, or increment $U$ |
| Sum | Math Operations | $T - mg$ (signs `\|+-`) |
| Gain | Math Operations | $1/m$ |
| Integrator (×2) | Continuous | Acceleration → velocity → altitude |
| Mux | Signal Routing | Combine signals for display |
| Scope | Sinks | View results |

### Absolute model

```text
Total Thrust (T)
       |
       v
   Sum (+, -) <--- Weight (mg)
       |
       v
    Gain 1/m
       |
       v
 Acceleration
       |
       v
 Integrator (v0)
       |
       v
   Velocity
       |
       v
 Integrator (z0)
       |
       v
   Altitude
```

### Deviation model

```text
Thrust Increment (U)
       |
       v
    Gain 1/m
       |
       v
 Acceleration
       |
       v
 Integrator (v0)
       |
       v
   Velocity
       |
       v
 Integrator (z0)
       |
       v
   Altitude
```

### Parameter values

| Signal | Value |
|---|---|
| Hover thrust $T_0 = mg$ | 19.62 N |
| Thrust increment $U$ | 0.4 N |
| Total thrust $T = T_0 + U$ | 20.02 N |
| Acceleration $U/m$ | 0.2 m/s² |

### Solver settings

| Setting | Value |
|---|---|
| Start time / Stop time | `0` / `4` |
| Solver type | Variable-step, `ode45` (matches the MATLAB script) |
| Relative tolerance | `1e-9` |
| Absolute tolerance | `1e-11` |
| Max step size | `0.01` |

For exactly the same time points as the MATLAB script, use a fixed-step solver (`ode4`) with a step size of `0.01`.

### Comparison procedure

1. Save a copy of the Day 2 model as `day03_absolute_vs_delta.slx`.
2. Set the absolute-model thrust to $T_0 + U$.
3. Add the deviation-model branch with input $U$.
4. Use the same initial velocity and altitude in both branches.
5. Connect both altitude outputs to a Mux and Scope.
6. Subtract the altitude outputs using a Sum block and send the result to a second Scope.
7. Observe whether the difference remains close to zero.
8. Repeat with nonzero initial conditions.

The Mux combines signals for display; it does not add them.

The altitude difference is:

$$e_z = z_{\text{total}} - z_{\text{deviation}}$$

Ideally $e_z = 0$. In practice it is zero only up to floating-point round-off (see below).

---

## 11. Error Analysis

Three different comparisons are made on Day 3. Each has a different expected error level.

| Comparison | Where | Expected size | Source |
|---|---|---|---|
| ODE vs. analytical solution | MATLAB (`err_z`, `err_v`) | Round-off level, far below the `1e-9` tolerance | Solver tolerance and floating point |
| Deviation ODE vs. total-thrust ODE | MATLAB (`err_models`) | Round-off level | Floating-point arithmetic |
| Absolute vs. deviation model | Simulink difference Scope | About $10^{-16}$ m in the run shown | Floating-point arithmetic |

### 11.1 MATLAB error

The script prints three values:

- `err_z`: the largest altitude difference between `ode45` and the closed-form solution.
- `err_v`: the largest velocity difference between `ode45` and the closed-form solution.
- `err_models`: the largest difference between the deviation model and the total-thrust model.

All three should be tiny. The acceleration is constant, so velocity is linear in time and altitude is quadratic. Runge-Kutta methods such as `ode45` integrate low-order polynomials essentially exactly, so the solver adds almost no truncation error. What remains is round-off, which is far below the requested tolerances (`RelTol = 1e-9`, `AbsTol = 1e-11`).

This result is specific to constant acceleration. For a nonlinear model such as quadratic rotor thrust or tilted thrust, `ode45` would show real numerical error that depends on the tolerance.

The error plot shows the same three quantities against time on a log axis. Expect values hugging the round-off floor, typically growing slightly with time as the altitude grows.

### 11.2 Simulink error

The difference between the absolute and deviation altitude outputs is plotted on a Scope. In the run shown, the vertical axis is scaled by $10^{-16}$ and the largest difference is about $1.3 \times 10^{-16}$ m, against a final altitude of 1.6 m. This is the expected result: the two models are mathematically identical because $T - mg = U$.

<!-- After saving the Scope screenshot, uncomment the next line:
![Simulink altitude difference](figures/day03_simulink_error.png)
-->

**Why it is not exactly zero.** The absolute model computes $(T_0 + U) - mg$, while the deviation model uses $U$ directly. In double-precision arithmetic (about 16 significant digits), $19.62 + 0.4 - 19.62$ does not return exactly $0.4$. The tiny discrepancy in the input propagates through both integrators.

**Why the curve looks like a staircase.** Near an altitude of 1 to 2 m, the smallest representable change in a double-precision number is about $2.2 \times 10^{-16}$. The difference can only change in multiples of that spacing, so it moves in small discrete steps instead of varying smoothly. The staircase is a round-off artifact, not a physical or modelling effect.

**Interpretation.** The two Simulink models agree to machine precision. A good way to state this in a report is: *the absolute and deviation models differ only at floating-point round-off level, about $10^{-16}$ m.*

### 11.3 Comparing Simulink with the closed-form solution

To check Simulink against the analytical result, log the altitude signal to the workspace (Scope settings → Logging → Log data to workspace) and compare it with `z_exact`. The snippet below is a template; adjust the variable name to match the signal you logged.

```matlab
% Template: assumes the Scope data was logged as 'simout' (Array format)
t_sim = simout.time;
z_sim = simout.signals.values(:,1);

z_ref = z0 + v0*t_sim + (U/(2*m))*t_sim.^2;   % closed form at Simulink's time points
err_simulink = max(abs(z_sim - z_ref));
fprintf('Maximum Simulink altitude error: %.3g m\n', err_simulink);
```

A variable-step solver chooses its own time points, so evaluate the closed-form solution at the logged times (as above) rather than at the MATLAB script's `t` vector.

### 11.4 What these errors do and do not tell you

- **Numerical error** (solver tolerance, step size) is small here and can be reduced further with tighter tolerances or smaller steps.
- **Round-off error** comes from finite-precision arithmetic. It cannot be removed by tightening solver tolerances, and it sets the floor you see on the plots.
- **Parameter and model errors** do not appear in these comparisons at all. The analytical solution and both ODEs use the same mass, gravity, and thrust values, so agreement here does not say anything about how well the model matches a real UAV.

---

## 12. Model Verification and Limitations

### Verification

Verification checks whether the equations have been implemented correctly.

For this project, verification includes:

- Comparing the ODE result against the analytical solution.
- Comparing the total-thrust and deviation models.
- Checking initial conditions and terminal values.
- Recording numerical errors and solver tolerances.

### Validation

Validation checks whether the mathematical model adequately represents a real UAV.

A model can be implemented and solved correctly but still be inaccurate for real flight if important physical effects are omitted.

### Error classification

| Error type | Example | Does a smaller time step fix it? |
|---|---|---|
| Numerical error | Approximation error from Euler integration | Can reduce it |
| Round-off error | Floating-point rounding (about $10^{-16}$) | No |
| Parameter error | Incorrect mass value | No |
| Model error | Neglecting drag when it matters | No |
| Implementation error | Incorrect gravity sign | No |
| Logging/plotting error | Displaying the wrong signal | No |

The simplified model assumes constant mass and gravity, constant thrust increment during the test, and no aerodynamic drag or wind. It also does not model actuator dynamics, attitude dynamics, or ground contact.

---

## 13. Key Takeaways

- An operating point is the nominal condition used as a reference; an initial condition describes the beginning of a simulation.
- Hover thrust is $T_0 = mg$.
- Total thrust can be expressed as $T = T_0 + u$.
- Substituting the hover operating point gives the deviation equation $\ddot z = u/m$.
- Gravity cancels from the ideal deviation equation because hover thrust balances weight.
- The second-order model can be represented as a first-order ODE using altitude and velocity as states.
- `ode45` numerically integrates the ODE, while the analytical solution provides a reference for verification.
- Forward Euler demonstrates how finite time steps introduce numerical error.
- The absolute and deviation models should agree when their inputs and initial conditions are defined consistently.
- Remaining differences of about $10^{-16}$ are floating-point round-off, not modelling error.
- Numerical accuracy does not guarantee physical accuracy.


