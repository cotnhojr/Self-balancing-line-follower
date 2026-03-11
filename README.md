# Self-Balancing Line Follower Robot

A two-wheel **self-balancing robot** capable of maintaining dynamic stability while following a predefined path using infrared sensors.

The robot combines **sensor fusion**, **optimal control**, and **embedded real-time motor control** techniques including:

* Kalman Filter for angle estimation
* LQR Controller for balance stabilization
* PID Control for line tracking
* Encoder feedback for motion monitoring

This project demonstrates practical implementation of **control theory in embedded robotics systems**.

---

# Robot Overview

This robot behaves like an **inverted pendulum system**.

It must constantly adjust motor torque to prevent falling while simultaneously navigating along a track.

Main capabilities:

* Self balancing on two wheels
* Line tracking using infrared sensors
* Closed-loop motor control
* Real-time sensor fusion

---

# Project Structure

```
self-balancing-line-follower
│
├── firmware
│   └── LQR_PID
│       ├── LQR_PID.ino
│       ├── i2c.ino
│       ├── kalman.h
│       ├── interrupte.h
│       └── io.h
│
├── hardware
│   └── (schematic, wiring diagrams)
│
├── images
│   └── (robot photos, system diagrams)
│
└── README.md
```

---

# Firmware Description

## LQR_PID.ino

Main program of the robot.

Responsibilities:

* IMU data acquisition
* Running Kalman filter
* LQR balance controller
* PID line tracking controller
* Motor PWM output
* Encoder feedback processing

This file coordinates the entire control loop.

---

## i2c.ino

Implements the **I2C communication interface**.

Used for:

* Communicating with the MPU6050 IMU
* Reading accelerometer and gyroscope data
* Configuring sensor registers

---

## kalman.h

Implements the **Kalman Filter** used to estimate the robot tilt angle.

The filter fuses:

* Accelerometer measurements
* Gyroscope angular velocity

Output values include:

* estimated angle
* angular velocity

Kalman filtering significantly improves stability compared to raw sensor data.

---

## interrupte.h

Handles **interrupt service routines** for wheel encoders.

Responsibilities:

* Counting encoder pulses
* Measuring wheel speed
* Providing feedback for the control system

Interrupt-based processing ensures accurate measurement.

---

## io.h

Defines **hardware pin configuration** and low-level I/O control.

Typical definitions include:

* Motor driver pins
* PWM outputs
* Encoder inputs
* Infrared sensor pins

Centralizing hardware configuration simplifies maintenance.

---

# Hardware Components

| Component       | Description                    |
| --------------- | ------------------------------ |
| Microcontroller | Arduino-compatible board       |
| IMU             | MPU6050                        |
| Motors          | DC gear motors                 |
| Motor Driver    | H-bridge driver                |
| Encoders        | Quadrature encoders            |
| Sensors         | Infrared line tracking sensors |
| Power Supply    | Lithium battery pack           |

---

# Control System

The robot uses multiple control techniques.

---

## Kalman Filter

Used for **sensor fusion**.

Combines:

* Accelerometer
* Gyroscope

Result:

Stable estimation of the robot tilt angle.

---

## LQR Controller

Balance stabilization is achieved using **Linear Quadratic Regulator (LQR)**.

The controller minimizes the cost function:

J = ∫(xᵀQx + uᵀRu) dt

Where:

* x = system state
* u = control input

This method provides stable and optimal control for the inverted pendulum system.

---

## PID Controller

PID control is used for **line tracking steering correction**.

The controller adjusts the difference between left and right motor speed to keep the robot centered on the line.

---

# System Control Loop

Typical control loop execution:

1. Read IMU sensor data
2. Estimate tilt angle using Kalman Filter
3. Compute LQR control signal
4. Read line sensor values
5. Compute PID steering correction
6. Generate PWM motor commands

This loop runs continuously in real time.

---

# Features

* Two-wheel self balancing robot
* Kalman filter based sensor fusion
* LQR optimal balance control
* PID line tracking control
* Encoder feedback system
* Real-time motor PWM control

---

# Images

Robot images and diagrams can be found in:

```
images/
```

Example content:

* robot_photo.jpg

---

# Possible Improvements

Future upgrades may include:

* Bluetooth remote control
* WiFi telemetry
* SLAM navigation
* ROS integration
* Mobile application interface
* Advanced path planning algorithms

---

# Learning Outcomes

This project demonstrates knowledge in:

* Embedded systems programming
* Robotics control systems
* Sensor fusion techniques
* Real-time motor control
* Interrupt-based embedded design

---

# Demo

Link Youtube: https://www.youtube.com/watch?v=jwksNqMpJ7w&t=690s

---

# License

MIT License
