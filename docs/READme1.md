# Day 1: Kinematics and the Constant-Acceleration Model

## Overview

This project introduces the mathematical foundations required to model the vertical motion of a multirotor Unmanned Aerial Vehicle (UAV).

The focus of Day 1 is **kinematics**: understanding how acceleration, velocity, and altitude are related through differentiation and integration.

A simple constant-acceleration model is implemented using MATLAB and Simulink. This establishes the foundation for the force-based UAV model developed in subsequent lessons.

## Objectives

- Understand functions, derivatives, integrals, and initial conditions.
- Distinguish altitude, velocity, and acceleration using their physical units.
- Implement the equations in MATLAB.
- Build a two-integrator Simulink model.
- Verify simulation results against analytical calculations.
- Understand the role of the plant in a control system.

## MATLAB Results

**Final velocity = 0.80 m/s**
**Final altitude = 1.60 m**

<img width="682" height="506" alt="image" src="https://github.com/user-attachments/assets/86321f0a-59d3-4828-9286-c2d8e63cc11b" />

## Equations

Let $V$ = velocity, $A$ = acceleration, $S$ = altitude, $T$ = time.

$$S(T) = S(0) + V\,T \quad \text{(constant velocity)}$$

$$S(T) = S(0) + V(0)\,T + \tfrac{1}{2} A T^2$$

$$V(T) = V(0) + A\,T$$

> [!NOTE]
> The acceleration equations above apply **only for constant acceleration**.

## Analytical Calculation

### Step 1: Calculate velocity

$$v(4) = 0 + (0.2)(4) = 0.8 \ \text{m/s}$$

### Step 2: Calculate altitude

$$z(4) = 0 + (0)(4) + 0.5\,(0.2)(4^2) = 1.6 \ \text{m}$$

### Expected final values

| Quantity | Value |
|----------|-------|
| Acceleration | 0.2 m/s² |
| Velocity | 0.8 m/s |
| Altitude | 1.6 m |

## Simulink Results

<img width="1401" height="417" alt="image" src="https://github.com/user-attachments/assets/a8ac8063-4a8b-4af8-aa7b-b9803d625d98" />
<img width="696" height="495" alt="image" src="https://github.com/user-attachments/assets/669e5c7f-40ee-41e5-aa82-be0f2fe5cae0" />
<img width="697" height="487" alt="image" src="https://github.com/user-attachments/assets/bc1c76dd-1177-4e04-84b5-2bf3b7779668" />
<img width="693" height="492" alt="image" src="https://github.com/user-attachments/assets/9f3699f1-aab6-4624-b4f7-e9b2b0317c2b" />

## Conclusion

The MATLAB and Simulink results match the analytical values (0.8 m/s and 1.6 m), so both implementations are verified.
