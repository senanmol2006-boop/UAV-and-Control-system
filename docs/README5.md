# Day 5: Equivalent Simulink Representations

Verifying that one simplified UAV vertical-motion model gives the same response when built as an **integrator chain**, a **transfer function**, and a **state-space** model in MATLAB Simulink.



## Objective

Model the vertical motion of a UAV near its hover position in three ways:

1. Integrator-based model
2. Transfer Function model
3. State-Space model

Different mathematical representations of the same system must produce the same output when parameters, inputs, initial conditions, and simulation settings are consistent. This experiment checks that claim numerically.

---

## Physical System

From Newton's second law, the simplified vertical motion near hover is:

```math
m\ddot{z}=u
```

| Symbol | Meaning | Unit |
|---|---|---|
| $`m`$ | Mass of the UAV | kg |
| $`z`$ | Vertical position (altitude) | m |
| $`\dot{z}`$ | Vertical velocity | m/s |
| $`\ddot{z}`$ | Vertical acceleration | m/s² |
| $`u`$ | Thrust **increment** relative to hover | N |

Rearranged:

```math
\boxed{\ddot{z}=\frac{u}{m}}
```

For the same thrust increment, a heavier UAV accelerates less than a lighter one.

> **Assumptions:** This model ignores aerodynamic drag, changing mass, and other complex flight dynamics. The input $`u`$ is the thrust increment relative to hover, not the total motor thrust.

---

## The Three Representations

### Model 1: Integrator-Based

Built directly from the differential equation:

```math
\dot{z}=\int\frac{u}{m}\,dt \qquad z=\int\dot{z}\,dt
```

```text
         Gain             Integrator          Integrator
u(t) ──► [1/m] ──► Acceleration ──► Velocity ──► Position z(t)
```

**Simulink blocks:** one Gain (`1/m`) and two Integrators, both with initial condition `0`.

**Why use it:** each stage has a clear physical meaning (acceleration, velocity, position), which makes it ideal when building from a physical differential equation.

### Model 2: Transfer Function

Laplace transform of $`m\ddot{z}=u`$ with zero initial conditions:

```math
ms^2Z(s)=U(s)
```

```math
\boxed{G(s)=\frac{Z(s)}{U(s)}=\frac{1}{ms^2}}
```

**Simulink block:** Transfer Fcn

- Numerator: `1`
- Denominator: `[m 0 0]`

For $`m=2`$: $`G(s)=\dfrac{1}{2s^2}`$, denominator `[2 0 0]`.

> **Do not add another `1/m` gain before the Transfer Fcn.** The mass is already in the denominator, so an extra gain would divide by the mass twice.

**Why use it:** input-output analysis, stability, frequency response, and classical control design.

**Limitation:** the standard form assumes zero initial conditions. Nonzero initial position or velocity must be handled separately or through state-space.

### Model 3: State-Space

State-space rewrites a higher-order equation as a set of **first-order** equations, one per *state*.

#### Step 1: Choose the states

A **state** is a quantity that, together with the future input, is enough to predict everything that happens next. Knowing the future thrust is not enough for a UAV. You also need to know **where it is now** and **how fast it is moving now**:

```math
x_1=z\ (\text{position}),\qquad x_2=\dot{z}=v\ (\text{velocity}),\qquad x=\begin{bmatrix}z\\v\end{bmatrix}
```

The number of states equals the order of the system. Second order means two states, which is also why the integrator model has two integrators.

#### Step 2: Write the first-order equations

We need $`\dot{x}_1`$ and $`\dot{x}_2`$ using only the states and the input.

- $`\dot{x}_1=\dot{z}=x_2`$ (the definition "velocity is the derivative of position")
- $`\dot{x}_2=\ddot{z}=\dfrac{u}{m}`$ (Newton's law, $`m\ddot{z}=u`$)

Writing the zero terms explicitly:

```math
\dot{x}_1=0\cdot x_1+1\cdot x_2+0\cdot u
```

```math
\dot{x}_2=0\cdot x_1+0\cdot x_2+\tfrac{1}{m}\cdot u
```

#### Step 3: Match to the standard form

```math
\boxed{\dot{x}=Ax+Bu}\qquad\boxed{y=Cx+Du}
```

Written out for two states:

```math
\begin{bmatrix}\dot{x}_1\\\dot{x}_2\end{bmatrix}=\begin{bmatrix}a_{11}&a_{12}\\a_{21}&a_{22}\end{bmatrix}\begin{bmatrix}x_1\\x_2\end{bmatrix}+\begin{bmatrix}b_1\\b_2\end{bmatrix}u
```

Multiplying out the rows gives:

```math
\dot{x}_1=a_{11}x_1+a_{12}x_2+b_1u
```

```math
\dot{x}_2=a_{21}x_1+a_{22}x_2+b_2u
```

Comparing coefficient by coefficient with Step 2:

| Row | Equation from Step 2 | Coefficients |
|---|---|---|
| 1 | $`\dot{x}_1=0\cdot x_1+1\cdot x_2+0\cdot u`$ | $`a_{11}=0,\ a_{12}=1,\ b_1=0`$ |
| 2 | $`\dot{x}_2=0\cdot x_1+0\cdot x_2+\frac{1}{m}u`$ | $`a_{21}=0,\ a_{22}=0,\ b_2=\frac{1}{m}`$ |

#### How $`A`$ was obtained

```math
A=\begin{bmatrix}0&1\\0&0\end{bmatrix}
```

The entry $`a_{ij}`$ means "how much state $`j`$ affects the rate of change of state $`i`$".

- $`a_{12}=1`$: velocity feeds directly into the rate of change of position. This is the only nonzero entry.
- $`a_{11}=0`$: position does not affect its own rate of change.
- $`a_{21}=0`$ and $`a_{22}=0`$: neither position nor velocity affects acceleration. There is no spring force (which would depend on $`z`$) and no drag (which would depend on $`v`$).

If drag with coefficient $`c`$ were added, $`m\ddot{z}=u-c\dot{z}`$ gives $`\dot{x}_2=-\frac{c}{m}x_2+\frac{u}{m}`$, so $`a_{22}`$ would become $`-c/m`$.

#### How $`B`$ was obtained

$`B`$ collects the coefficient of $`u`$ in each row:

```math
B=\begin{bmatrix}0\\1/m\end{bmatrix}
```

Thrust does not change position directly ($`b_1=0`$). A force changes position only through velocity: force produces acceleration, acceleration changes velocity, and velocity changes position. So the thrust enters only the acceleration equation, scaled by $`1/m`$.

#### How $`C`$ and $`D`$ were obtained

$`y=Cx+Du`$ defines what is measured. With altitude as the output:

```math
y=z=1\cdot x_1+0\cdot x_2+0\cdot u\ \Rightarrow\ C=\begin{bmatrix}1&0\end{bmatrix},\quad D=0
```

- For velocity as the output: $`C=[0\ \ 1]`$.
- For both outputs: $`C=\begin{bmatrix}1&0\\0&1\end{bmatrix}`$.
- $`D=0`$ because thrust cannot change altitude instantly. It must pass through two integrations first.

| Matrix | Role |
|---|---|
| $`A`$ | State dynamics |
| $`B`$ | How the input affects the states |
| $`C`$ | Maps states to the output |
| $`D`$ | Direct input-to-output feedthrough |

#### Simulink implementation

**Block:** State-Space

```matlab
A = [0 1; 0 0];
B = [0; 1/m];
C = [1 0];
D = 0;
x0 = [0; 0];   % [initial position; initial velocity]
```

Because $`x=[z;v]`$, the first row of $`A`$ and $`B`$ belongs to position and the second to velocity, so $`x_0=[z_0;\,v_0]`$ must follow the same order.

#### Link to the integrator diagram

The state-space equations are the integrator model in matrix form:

```text
u ──► [1/m] ──► ẋ₂ ──∫──► x₂ (=v) ──┬──∫──► x₁ (=z) ──► y
                                     │
                          (this is ẋ₁ = x₂, the "1" in A)
```

- The $`1/m`$ gain is the entry in $`B`$.
- Each integrator turns a state derivative into a state.
- The "1" in $`A`$ is the wire from the first integrator's output (velocity) to the second integrator's input.

#### Check: it matches the transfer function

The link between the two forms is $`G(s)=C(sI-A)^{-1}B+D`$:

```math
sI-A=\begin{bmatrix}s&-1\\0&s\end{bmatrix},\qquad(sI-A)^{-1}=\frac{1}{s^2}\begin{bmatrix}s&1\\0&s\end{bmatrix}
```

```math
G(s)=[1\ \ 0]\cdot\frac{1}{s^2}\begin{bmatrix}s&1\\0&s\end{bmatrix}\begin{bmatrix}0\\1/m\end{bmatrix}=\frac{1}{s^2}[1\ \ 0]\begin{bmatrix}1/m\\s/m\end{bmatrix}=\frac{1}{ms^2}
```

This is exactly the transfer function from Model 2, confirming that the representations describe the same system.

**Why use it:** MIMO systems, modern control design, multiple interacting states, and explicit nonzero initial conditions.

---

## Comparison

| Feature | Integrator-Based | Transfer Function | State-Space |
|---|---|---|---|
| Main idea | Build from physical operations | Input-output relationship | Evolution of internal states |
| Mathematical basis | Differential equations | Laplace-domain equations | First-order state equations |
| Main Simulink blocks | Gain + Integrators | Transfer Fcn | State-Space |
| Shows velocity explicitly | Yes | Not directly | Yes, as a state |
| Initial conditions | Integrator settings | Standard form assumes zero | Initial state vector |
| Best for | Physical understanding | Classical control analysis | Complex systems, modern control |

---

## Simulation Setup

The experiment uses a delayed step input:

```math
u(t)=0.4\,H(t-1)=\begin{cases}0, & t<1\\0.4, & t\geq1\end{cases}
```

```matlab
m = 2;
U = 0.4;
t_step = 1;
z0 = 0;
v0 = 0;
```

Simulation time: $`t=0`$ s to $`t=6`$ s.

**Step block settings**

| Parameter | Value |
|---|---|
| Step time | `t_step` |
| Initial value | `0` |
| Final value | `U` |
| Sample time | `0` (continuous) |

**Expected behaviour:** before $`t=1`$ s everything stays at zero. After the step:

```math
\boxed{a=\frac{0.4}{2}=0.2\ \text{m/s}^2}
```

Velocity rises linearly and altitude rises quadratically. Altitude and velocity stay continuous at the step, while acceleration jumps abruptly.

---

## Analytical Reference

For a delayed step and zero initial conditions:

```math
z(t)=\begin{cases}0, & t<1\\[4pt]\dfrac{U}{2m}(t-1)^2, & t\geq1\end{cases}
```

With $`U=0.4`$ N and $`m=2`$ kg: $`z(t)=0.1\,(t-1)^2`$ for $`t\geq1`$.

```matlab
z_ref = U * max(t - t_step, 0).^2 / (2*m);
```

**Checkpoints**

| Time | Altitude $`z`$ | Velocity $`v`$ |
|---|---:|---:|
| 0 s | 0 m | 0 m/s |
| 1 s | 0 m | 0 m/s |
| 2 s | 0.1 m | 0.2 m/s |
| 3 s | 0.4 m | 0.4 m/s |
| 6 s | 2.5 m | 1.0 m/s |

---

## Verifying Equivalence

The goal is not just three plots, but proof that all three implementations represent the same system.

1. **Record outputs** with To Workspace blocks: `z_int`, `z_tf`, `z_ss` (format `Timeseries`, decimation `1`, "limit data points" disabled).
2. **Compare outputs** by plotting all three altitude responses on the same axes.
3. **Compare against the analytical reference.** Three models can agree with each other and still share the same mistake (for example, a wrong mass).
4. **Compute the maximum absolute error** for each model:

```math
e(t)=z_{\text{sim}}(t)-z_{\text{ref}}(t)\qquad\boxed{E_{\infty}=\max_t|e(t)|}
```

**Acceptance criterion:**

```math
\boxed{E_{\infty}<10^{-5}\ \text{m}}
```

This is a numerical verification target for the simulation, not a claim about real UAV sensor accuracy.

**Why residuals?** Overlapping curves can hide small discrepancies. Plotting the residual $`e(t)`$ exposes them.

---

## Solver Settings and Time

| Setting | Value |
|---|---|
| Solver | `ode45` |
| Relative tolerance | `1e-8` |
| Absolute tolerance | `1e-10` |
| Maximum step size | `0.01` s |
| Start time | `0` s |
| Stop time | `6` s |

Three different meanings of "time":

1. **Solver time:** when the numerical solver computes the response.
2. **Logging time:** when data is recorded for analysis.
3. **Controller sample time:** how often a discrete controller updates.

These are related but not necessarily equal. If the logged time vectors differ between models, interpolate onto a common time grid before computing point-by-point errors.

---

## Common Mistakes

| # | Mistake | Fix |
|---|---|---|
| 1 | Applying the mass factor twice (`Gain 1/m` → `Transfer Fcn 1/(m s²)`) | Use either gain + integrators, or the transfer function containing the mass |
| 2 | Wrong transfer function coefficients | Denominator must be `[m 0 0]` (for $`m=2`$: `[2 0 0]`) |
| 3 | Wrong state ordering | If `x = [z; v]`, then `x0 = [z0; v0]` |
| 4 | Inconsistent initial conditions | All three models must start with the same position and velocity |
| 5 | Comparing outputs at different times | Interpolate onto a common time grid first |
| 6 | Trusting overlapping plots alone | Also check the analytical solution and residuals |
| 7 | Confusing model workspace and base workspace | A model workspace variable can take precedence; check where parameters are defined |

---

## Project Structure

```text
Day-05/
├── v1_equivalent.slx
├── day05_compare_models.m
├── results/
│   └── day05.mat
└── figures/
    ├── altitude_comparison.png
    └── residual_comparison.png
```

The MATLAB script `day05_compare_models.m` should:

1. Run the Simulink model.
2. Retrieve the three altitude outputs.
3. Align the outputs on a common time grid.
4. Compute the analytical reference response.
5. Plot the altitude responses.
6. Plot the residuals.
7. Calculate the maximum absolute error for each model.
8. Check each model against the $`10^{-5}`$ m criterion.

### Running

1. Open `v1_equivalent.slx` in MATLAB/Simulink.
2. Run `day05_compare_models.m`.
3. Find the plots in `figures/` and the saved data in `results/day05.mat`.

---

## Revision Questions

<details>
<summary>Click to expand questions</summary>

1. What is the physical meaning of the gain $`1/m`$?
2. Why are two integrators required to get altitude from acceleration?
3. How do we derive $`G(s)=1/(ms^2)`$?
4. Why is the Transfer Fcn denominator `[m 0 0]`?
5. What do the matrices $`A`$, $`B`$, $`C`$, and $`D`$ represent?
6. If the UAV mass increases while the thrust increment stays constant, what happens to acceleration?
7. Why check the three models against an analytical solution?
8. Why can the three curves overlap even when the implementation is wrong?
9. What is the difference between solver time, logging time, and controller sample time?
10. For a 2 kg UAV receiving a 0.4 N thrust increment at $`t=1`$ s, what is its altitude at $`t=3`$ s?

</details>

<details>
<summary>Click to expand answers</summary>

1. It converts thrust increment into acceleration.
2. Acceleration is integrated once for velocity and again for position.
3. The Laplace transform of $`m\ddot{z}=u`$ under zero initial conditions gives $`ms^2Z(s)=U(s)`$.
4. The denominator represents $`ms^2+0s+0`$.
5. $`A`$ is the state dynamics, $`B`$ is the input-to-state map, $`C`$ maps states to output, $`D`$ is direct feedthrough.
6. Acceleration decreases.
7. The analytical solution is an independent reference for correctness.
8. They may share the same wrong parameter, input, or modelling assumption.
9. Solver time concerns numerical calculation, logging time concerns recorded data, and controller sample time concerns discrete controller updates.
10. $`0.4`$ m.

</details>

---

## Takeaway

One physical system can have several mathematical representations, and each offers a different view:

```math
\boxed{m\ddot{z}=u}
```

- **Integrators** show the physical sequence from acceleration to position.
- **Transfer functions** describe the input-output relationship.
- **State-space** describes the system through its internal states.

When implemented consistently, all three agree. Checking against an analytical solution, inspecting residuals, and computing the maximum error turns a working simulation into a **verified model**.

<img width="963" height="380" alt="image" src="https://github.com/user-attachments/assets/2d0f8d7a-041b-4bd2-a120-b1caf7988688" />
<img width="211" height="75" alt="image" src="https://github.com/user-attachments/assets/35fdb788-31a2-434a-8691-3210d5755a85" />

---

### Suggested commit message

```text
Day 5: Verify integrator, transfer function, and state-space equivalence
```
