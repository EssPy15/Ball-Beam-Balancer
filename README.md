# Ball–Beam Balance System (PID Control using Arduino)

## Overview

This project involves the design, fabrication, and control of a **Ball–Beam Balance System**, a classical **open-loop unstable control problem** widely used to demonstrate feedback control concepts. The objective is to regulate the position of a ball rolling on a beam by adjusting the beam angle using a servo motor, based on real-time sensor feedback.

The project integrates **mechanical design**, **embedded systems**, and **control theory**, and was implemented using **Arduino Uno**, an ultrasonic distance sensor, and a servo motor.

---

## Objectives

* Design and construct a physical ball–beam setup with appropriate mechanical constraints.
* Measure real-time ball position using an ultrasonic sensor.
* Implement a **closed-loop control system** using a **PID controller**.
* Tune controller gains to achieve stability, fast response, and minimal steady-state error.
* Analyze system behavior under varying controller parameters and noise conditions.

---

## System Description

### Mechanical Setup

* Beam length: **42 cm**, fabricated using layered cardboard for rigidity.
* Actuation: **Servo motor** connected via a lever mechanism to control beam tilt.
* Ball motion: 1-DOF rolling motion along the beam under gravity.

### Sensing

* **Ultrasonic sensor** placed to measure the instantaneous position of the ball.
* Sensor alignment optimized to ensure reflections from the center of the ball.

---

## Control Architecture

* **Control Type:** Closed-loop feedback control
* **Setpoint:** Ball position at the balanced (horizontal) beam position
* **Error Signal:** Difference between reference position and measured ball position

### Controller

* Started with **Proportional (P) control** to study basic stability and response.
* Extended to **PID control** to reduce steady-state error and oscillations.
* Gains tuned experimentally using serial plot analysis.

### Filtering

* Applied a **moving average filter** (and evaluated Kalman filtering) to smooth noisy ultrasonic sensor data.
* Filtering resulted in improved control stability and reduced actuator jitter.

---

## Performance & Results

* Achieved **stable ball positioning within ±1–2 cm** of the reference point.
* Reduced **settling time by ~40–50%** compared to P-only control and achieved it under 2 secs.
* Reduced sensor noise by **~30%** using filtering techniques.
* Observed system instability beyond a critical **Kp**, validating theoretical control concepts.

---

## Experimental Validation

* Used **Arduino Serial Plotter** to analyze ball position, control signal, and oscillations.
* Studied the effect of varying **Kp, Ki, and Kd** on system stability and performance.
* Compared sensor outputs **with and without filtering**.

---

## Key Learnings

* Practical challenges in controlling unstable systems.
* Relationship between controller gains and system stability.
* Importance of sensor filtering in real-time embedded control.
* Bridging theoretical control concepts with real hardware implementation.

---

## Tools & Technologies

* **Arduino Uno**
* **Servo Motor**
* **Ultrasonic Distance Sensor**
* Embedded C / Arduino IDE
* Control Systems (PID, stability analysis)
* Signal filtering techniques

---

## Applications

* Control systems education
* Robotics and automation
* Foundation for advanced topics such as state-space control and observer design

---
