# Day 4: Laplace Transform and Simulink Modeling

## Overview

Day 4 connects differential equations, transfer functions, system responses, and MATLAB/Simulink simulation to build and verify a basic dynamic model of UAV altitude.

## 1. Laplace Transform

The Laplace transform converts differential equations from the time domain into the $s$-domain. This makes it easier to analyze dynamic systems and derive transfer functions.

## 2. Transfer Function

A transfer function describes the relationship between a system's input and output in the Laplace domain, assuming **zero initial conditions**.

For the UAV altitude model:

$$
m\ddot{z} = u(t)
$$

the transfer function is:

$$
G(s) = \frac{Z(s)}{U(s)} = \frac{1}{ms^2}
$$

## 3. Double Integrator

The model shows how a force increment produces acceleration, which integrates into velocity and then altitude.

Because this ideal model has no damping or restoring force, a constant force increment makes altitude grow **quadratically** with time.

## 4. Poles and Zeros

- **Poles:** roots of the transfer function's denominator.
- **Zeros:** roots of the numerator.

The double-integrator model has **two poles at the origin** and **no finite zeros**.

## 5. Step and Impulse Responses

| Response | Description | Double-integrator result |
|---|---|---|
| Step | Constant input applied suddenly | Altitude increases quadratically |
| Impulse | Brief idealized input | Velocity changes instantly, then altitude changes linearly with time |

## 6. Simulink Modeling

The system was built from these blocks:

| Block | Purpose |
|---|---|
| Step | Generates the force input |
| Transfer Fcn | Represents the UAV altitude dynamics |
| Scope | Displays altitude against time |
| To Workspace | Saves simulation data for analysis in MATLAB |

## 7. Model Verification

The simulated altitude response was compared with the theoretical solution:

$$
z(t) = \frac{U}{2m}\max(t - t_0,\, 0)^2
$$

where:

- $U$ is the force increment,
- $t_0$ is the input start time,
- initial altitude and velocity are zero.

## Key Takeaway

Day 4 linked the theory (Laplace transform, transfer function, poles and zeros) to simulation (Simulink and MATLAB) and verified that the simulated response matches the analytical result.
