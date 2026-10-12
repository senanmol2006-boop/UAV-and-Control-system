# Day 2 — Forces and Hover in UAV Control Engineering



---

## Overview

Day 2 focuses on understanding the forces acting on a UAV (Unmanned Aerial Vehicle) and deriving its vertical equation of motion using Newton's second law. The concepts are implemented in MATLAB and Simulink to observe how changes in thrust affect acceleration, velocity, and altitude.

The primary objective is to establish a mathematical model of vertical UAV motion that can later be used for control-system design and simulation.

## Learning Objectives

By the end of Day 2, I aim to understand:

- The forces acting on a UAV during vertical flight.
- The difference between thrust, weight, and net force.
- How to derive the equation of motion using Newton's second law.
- The physical meaning of hover and hover thrust.
- How thrust variations affect vertical acceleration.
- How to simulate the UAV's vertical motion using MATLAB.
- How to construct a basic vertical-motion model in Simulink.
- How initial velocity and altitude affect the resulting motion.

---

## 1. Reference Frames and Sign Convention

To model the UAV's vertical motion, a reference frame and sign convention must be established.

For this model:

- The vertical $z$-axis is positive upward.
- Thrust acts upward.
- Gravitational force acts downward.
- Altitude is represented by $z$.
- Vertical velocity is represented by $\dot{z}$.
- Vertical acceleration is represented by $\ddot{z}$.

The relationships between position, velocity, and acceleration are:

$$
v_z = \dot{z} = \frac{dz}{dt}
$$

$$
a_z = \ddot{z} = \frac{d^2z}{dt^2}
$$

> **Key idea:** The chosen sign convention determines the signs of the forces and acceleration in the equations.

## 2. Forces Acting on a UAV

The simplified vertical-motion model considers two forces.

| Force  | Symbol | Direction | Magnitude |
|--------|:------:|:---------:|:---------:|
| Thrust | $T$    | Upward    | $T$       |
| Weight | $W$    | Downward  | $mg$      |

Where:

- $T$ = total upward thrust in newtons (N).
- $m$ = UAV mass in kilograms (kg).
- $g$ = gravitational acceleration, approximately $9.81\ \text{m/s}^2$.
- $W = mg$ = gravitational force or weight in newtons.

The net vertical force is therefore:

$$
F_{\text{net}} = T - mg
$$

This equation describes the difference between the upward thrust and downward gravitational force.

## 3. Deriving the Equation of Motion

Newton's second law states:

$$
\sum F = ma
$$

Applying it to the UAV's vertical motion:

$$
T - mg = ma_z
$$

Solving for acceleration:

$$
\boxed{a_z = \frac{T - mg}{m}}
$$

Since $a_z = \ddot{z}$, the equation of motion becomes:

$$
\boxed{\ddot{z} = \frac{T}{m} - g}
$$

This is the fundamental equation used in the Day 2 vertical-motion model.

### Physical Interpretation

| Condition  | Result                                   |
|------------|------------------------------------------|
| $T > mg$   | Positive acceleration, directed upward   |
| $T = mg$   | Zero vertical acceleration               |
| $T < mg$   | Negative acceleration, directed downward |

> Zero acceleration does not necessarily mean the UAV is stationary. It may continue moving upward or downward at constant velocity.

---

## 4. Understanding Hover

Hover occurs when the UAV maintains a constant altitude with zero vertical velocity and zero vertical acceleration.

For steady vertical hover:

$$
T - mg = 0
$$

Therefore, the required total hover thrust is:

$$
\boxed{T_{\text{hover}} = mg}
$$

For an ideal quadcopter with four rotors sharing the load equally:

$$
\boxed{T_{\text{rotor}} = \frac{mg}{4}}
$$

### Example

Consider a UAV with a mass of $2\ \text{kg}$.

Given:

$$
m = 2\ \text{kg}, \qquad g = 9.81\ \text{m/s}^2
$$

Total hover thrust:

$$
T_{\text{hover}} = 2(9.81) = 19.62\ \text{N}
$$

Thrust per rotor:

$$
T_{\text{rotor}} = \frac{19.62}{4} = 4.905\ \text{N}
$$

Thus, the UAV requires an ideal total thrust of $19.62\ \text{N}$, or $4.905\ \text{N}$ per rotor when the load is equally distributed.

## 5. Thrust Variation and Vertical Motion

The UAV's acceleration depends on the difference between its total thrust and weight.

Using the same $2\ \text{kg}$ UAV:

| Total thrust | Net force | Acceleration |
|-------------:|----------:|-------------:|
| 19.22 N      | −0.40 N   | −0.20 m/s²   |
| 19.62 N      | 0 N       | 0 m/s²       |
| 20.02 N      | +0.40 N   | +0.20 m/s²   |

A small change in thrust around the hover value produces a corresponding change in vertical acceleration.

### Constant-Acceleration Equations

For constant thrust, the acceleration is constant under this simplified model.

Velocity:

$$
v_z(t) = v_0 + a_z t
$$

Altitude:

$$
z(t) = z_0 + v_0 t + \frac{1}{2} a_z t^2
$$

Where:

- $v_0$ = initial vertical velocity.
- $z_0$ = initial altitude.
- $t$ = elapsed time.

These equations allow the UAV's vertical motion to be calculated analytically and compared with the simulation.

---

## 6. Effect of UAV Tilt

When a UAV tilts, its thrust is no longer entirely vertical.

If the thrust vector has magnitude $T$ and is tilted by an angle $\theta$ from the vertical, its vertical component is:

$$
\boxed{T_z = T\cos\theta}
$$

The vertical equation of motion becomes:

$$
\boxed{a_z = \frac{T\cos\theta - mg}{m}}
$$

Consequently, the vertical component of thrust decreases as the tilt angle increases, assuming the total thrust magnitude remains constant.

This explains why a tilted UAV may lose altitude if its thrust is insufficient to balance its weight vertically.

---

## 7. MATLAB Implementation

MATLAB is used to calculate the UAV's net force, acceleration, velocity, and altitude under different thrust conditions.

### Simulation Procedure

1. Define the UAV mass and gravitational acceleration.
2. Calculate the hover thrust.
3. Specify thrust values above, below, and equal to hover thrust.
4. Calculate the net force and vertical acceleration.
5. Compute velocity and altitude using the constant-acceleration equations.
6. Plot the resulting motion to compare the different cases.

### Core Equations in MATLAB

```matlab
% UAV parameters
m = 2;
g = 9.81;

% Initial conditions
z0 = 0;
v0 = 0;

% Total thrust
T = 20.02;

% Calculate vertical motion
F_net = T - m*g;
a_z = F_net/m;

% Time vector
t = 0:0.01:4;

% Velocity and altitude
v_z = v0 + a_z*t;
z = z0 + v0*t + 0.5*a_z*t.^2;

% Display results
fprintf('Net force: %.2f N\n', F_net);
fprintf('Acceleration: %.2f m/s^2\n', a_z);

% Plot results
figure;

subplot(2,1,1);
plot(t, v_z);
grid on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Vertical Velocity');

subplot(2,1,2);
plot(t, z);
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Vertical Altitude');
```

This example uses constant thrust, constant mass, and zero initial velocity and altitude. The results should be consistent with the analytical equations.

---

## 8. Simulink Implementation

The same mathematical model can be represented using Simulink blocks.

### Block Diagram Structure

```text
             +----------------+
Thrust T --->|                |
             |   Sum (+, -)   |---> Net Force
Weight mg -->|                |          |
             +----------------+          v
                                      Gain 1/m
                                          |
                                          v
                                     Acceleration
                                          |
                                          v
                                      Integrator
                                     Initial v0
                                          |
                                          v
                                       Velocity
                                          |
                                          v
                                      Integrator
                                     Initial z0
                                          |
                                          v
                                        Altitude
```

### Required Blocks

| Block        | Purpose                             |
|--------------|-------------------------------------|
| Constant     | Supplies total thrust $T$           |
| Constant     | Supplies weight $mg$                |
| Sum          | Calculates $T - mg$                 |
| Gain         | Multiplies net force by $1/m$       |
| Integrator 1 | Converts acceleration into velocity |
| Integrator 2 | Converts velocity into altitude     |
| Scope        | Displays selected signals           |

### Important Configuration

- Configure the Sum block with signs `+-`.
- Set the Gain block to `1/m`.
- Set the first Integrator's initial condition to $v_0$.
- Set the second Integrator's initial condition to $z_0$.
- Connect Scopes to the signals you want to observe.

The model follows the chain:

$$
\text{Net Force} \rightarrow \text{Acceleration} \rightarrow \text{Velocity} \rightarrow \text{Altitude}
$$

> **Important:** Integrators represent the mathematical relationship between acceleration, velocity, and position. They do not independently determine whether the UAV is physically able to fly.

---

## 9. Analytical and Simulation Verification

A useful engineering practice is to verify simulation results against analytical calculations.

For example, let:

$$
m = 2\ \text{kg}, \quad T = 19.22\ \text{N}
$$

Then:

$$
a_z = \frac{19.22 - 19.62}{2} = -0.20\ \text{m/s}^2
$$

For $v_0 = 0$, $z_0 = 0$, and $t = 4\ \text{s}$:

$$
v_z = 0 + (-0.20)(4) = -0.80\ \text{m/s}
$$

$$
z = 0 + 0 + \frac{1}{2}(-0.20)(4^2) = -1.60\ \text{m}
$$

The mathematical model predicts a downward velocity of $0.80\ \text{m/s}$ and a vertical displacement of $-1.60\ \text{m}$.

> A simple integrator model may predict altitude below ground level. This is a limitation of the model: ground contact and collision constraints have not been implemented.

---

## 10. Practice Problems and Key Results

### Problem 1 — Hover Thrust

A UAV has a mass of $1.5\ \text{kg}$. Calculate its weight and the ideal thrust required per rotor for a four-rotor UAV hovering with equal rotor loading.

**Results:**

- Weight: $14.715\ \text{N}$
- Total hover thrust: $14.715\ \text{N}$
- Thrust per rotor: $3.67875\ \text{N}$

### Problem 2 — Vertical Motion

A $2\ \text{kg}$ UAV starts at an altitude of $2\ \text{m}$, with an initial upward velocity of $0.5\ \text{m/s}$. Its thrust is $19.22\ \text{N}$. Calculate its velocity and altitude after $2\ \text{s}$.

**Results:**

- Acceleration: $-0.20\ \text{m/s}^2$
- Final velocity: $0.10\ \text{m/s}$
- Final altitude: $2.60\ \text{m}$

Although the acceleration is downward, the UAV is still moving upward at the end of the interval.

### Problem 3 — Tilted Thrust

A $2\ \text{kg}$ UAV produces $20\ \text{N}$ of thrust at a tilt angle of $30^\circ$ from the vertical.

**Results:**

- Vertical thrust: approximately $17.32\ \text{N}$
- Vertical acceleration: approximately $-1.15\ \text{m/s}^2$

The UAV accelerates downward because its vertical thrust component is less than its weight.

---

## 11. Model Assumptions and Limitations

The model developed on Day 2 is intentionally simplified.

It assumes:

- Motion is restricted to the vertical direction.
- UAV mass remains constant.
- Gravitational acceleration is constant.
- Thrust changes are applied instantaneously.
- Aerodynamic drag and wind are neglected.
- Attitude is fixed unless a tilt angle is explicitly considered.
- Ground contact and collision constraints are not modeled.

These assumptions make the model useful for understanding basic UAV dynamics, but it is not yet a complete flight model.

In particular, realistic UAV control will eventually require consideration of actuator response, aerodynamic effects, attitude dynamics, and feedback control.

## 12. Key Takeaways

- A UAV's vertical motion is determined by the difference between thrust and weight.
- The vertical equation of motion is $\ddot{z} = T/m - g$ for a vertically aligned thrust vector.
- Hover requires thrust equal to weight when the UAV is stationary at a constant altitude.
- Acceleration and velocity are different: downward acceleration does not necessarily imply downward velocity.
- MATLAB provides a way to calculate and visualize the model's response.
- Simulink represents the dynamics using force summation, a gain, and two integrators.
- Analytical calculations help verify the simulation.
- Simplified models have limitations, particularly when representing ground contact and real flight behavior.

## Day 2 Deliverables

- [ ] Structured notes on forces, reference frames, and hover.
- [ ] Derivation of the vertical equation of motion.
- [ ] MATLAB simulation for different thrust values.
- [ ] Plots of vertical velocity and altitude.
- [ ] Simulink implementation of the vertical-motion model.
- [ ] Verification of simulation results against analytical calculations.
- [ ] Practice problems and observations documented.

## Next Steps

The next stage is to build on this basic physical model and explore how a UAV can be controlled to follow a desired altitude or vertical-velocity command. This provides the foundation for studying feedback control and more realistic UAV dynamics.
