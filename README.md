# 🏎️ High-Speed PID Line Follower Car

[![Arduino](https://img.shields.io/badge/Platform-Arduino_Nano-00979D?style=for-the-badge&logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![Control](https://img.shields.io/badge/Algorithm-PID_Control-red?style=for-the-badge)](https://en.wikipedia.org/wiki/PID_controller)
[![Hardware](https://img.shields.io/badge/Driver-L298N-blue?style=for-the-badge)](https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf)
[![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)](LICENSE)

<div align="center">
  <img src="images/robot_demo.gif" alt="PID Robot Demo" width="100%">

  <h3 align="center">Autonomous Trajectory Correction System</h3>

  <p align="center">
    A closed-loop control system implemented on an Arduino Nano to achieve high-speed and precise line tracking using the PID algorithm.
    <br />
    
  </p>
</div>

---

## 📖 Executive Summary

This project focuses on the implementation of a **Proportional-Integral-Derivative (PID)** controller for an autonomous mobile robot. Unlike simple "bang-bang" (left-right) controllers, this system calculates the error value from the **QTR-8A Analog Sensor Array** and continuously adjusts the motor speeds via PWM signals.

The result is a smooth, oscillating-free trajectory tracking, capable of handling sharp turns and variable speeds. The system features a robust power management unit using **LM2596** to ensure stable logic voltage despite motor load fluctuations.

## ⚙️ Hardware Specifications

The robot is built on a custom chassis with high-traction silicon tires for minimal slip.

| Component | Model | Purpose |
| :--- | :--- | :--- |
| **Microcontroller** | Arduino Nano (ATmega328P) | Central Processing Unit |
| **Sensor Array** | Pololu QTR-8A (Analog) | High-resolution line detection (8-channel) |
| **Motor Driver** | L298N Module | Dual H-Bridge for DC Motor Control |
| **Actuators** | N20 Micro Metal Gearmotors | High torque/RPM ratio propulsion |
| **Power Mgmt** | LM2596 Buck Converter | Regulating 7.4V to stable 5V for Logic |
| **Power Source** | 2x 3.7V 2400mAh Li-ion | High capacity energy storage (Series Connection) |
| **Traction** | High-Grip Silicon Wheels | Reducing slippage for better PID response |

## 🧮 PID Control Implementation

The core of this project is the PID algorithm which minimizes the error between the robot's current position and the center of the line.

The error is calculated using a weighted average of the 8 analog sensor values:
$$Error = \text{Target Position} - \text{Weighted Sensor Average}$$

The control output (correction signal) is computed as:

$$Output = (K_p \cdot e(t)) + (K_i \cdot \int e(t) dt) + (K_d \cdot \frac{de(t)}{dt})$$

* **Proportional (Kp):** Reacts to the current error. Pushes the robot back towards the line.
* **Integral (Ki):** Accumulates past errors. Used to eliminate steady-state error (e.g., if the robot drifts slightly to one side continuously).
* **Derivative (Kd):** Predicts future error based on the rate of change. This creates a "damping" effect, reducing oscillations and preventing overshooting on sharp turns.

> **Note:** The Kp and Kd gains were determined experimentally through iterative testing, while the Ki term is dynamically adjusted to prevent "Integral Windup."

## 🔌 Circuit Diagram & Pinout

The power distribution is critical. The L298N is powered directly from the batteries, while the Arduino and Sensors are powered via the LM2596 to prevent brown-outs.

| Module | Pin | Arduino Pin | Description |
| :--- | :--- | :--- | :--- |
| **L298N** | ENA | D3 (PWM) | Motor A Speed |
| **L298N** | IN1/IN2 | D4 / D5 | Motor A Direction |
| **L298N** | IN3/IN4 | D6 / D7 | Motor B Direction |
| **L298N** | ENB | D9 (PWM) | Motor B Speed |
| **QTR-8A**| A0-A7 | A0-A7 | Analog Sensor Outputs |
| **LM2596**| OUT+ | 5V / VIN | Regulated Power |

## 💻 Software & Tuning

### Experimental Tuning Process
1.  **Set Ki & Kd to 0.** Increase **Kp** until the robot follows the line but oscillates (wobbles).
2.  **Increase Kd** (Derivative) to dampen the oscillations and smooth the movement.
3.  **Add small Ki** (Integral) only if the robot fails to center itself perfectly on straight lines or complex curves.

```cpp
// Pseudocode for PID Loop
void loop() {
    int position = qtr.readLine(sensorValues);
    int error = position - 3500; // Center position

    int P = error;
    int I = I + error;
    int D = error - lastError;

    int PID_value = (Kp * P) + (Ki * I) + (Kd * D);
    
    // Adjust Motor Speeds
    int leftMotorSpeed = baseSpeed + PID_value;
    int rightMotorSpeed = baseSpeed - PID_value;
    
    setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
    lastError = error;
}
