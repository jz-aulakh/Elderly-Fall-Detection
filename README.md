# Elderly Fall Detection System

## Overview
This project presents the design and implementation of an automated elderly fall detection system using inertial sensor data. The system continuously monitors human motion and detects fall events without requiring any manual user input. It is intended as an academic and experimental implementation focusing on reliability, low cost, and reduced false alarms.

The system is implemented on an Arduino microcontroller and uses data from the MPU6050 accelerometer and gyroscope. A rule-based, multi-stage fall detection algorithm is used to distinguish real falls from normal daily activities.

---

## Motivation
Falls are one of the most common causes of serious injury among elderly individuals, especially those living alone. In many cases, manual emergency alert systems fail because the person may be unconscious or unable to react after a fall.

This project aims to address this problem by automatically detecting fall events based on physical motion characteristics such as freefall, impact, and post-fall inactivity. The focus is on understanding sensor behavior, signal thresholds, and real-time decision logic in embedded systems.

---

## System Architecture
The system consists of the following main units:

1. **Sensing Unit**  
   - MPU6050 accelerometer and gyroscope  
   - Measures linear acceleration and angular velocity

2. **Processing Unit**  
   - Arduino microcontroller  
   - Executes a rule-based fall detection algorithm in real time

3. **Alert / Output Unit**  
   - Serial output for debugging and monitoring  
   - (Alert mechanisms such as GSM can be integrated in future versions)

---

## Working Principle

### Sensor Data Acquisition
The MPU6050 provides:
- Acceleration along X, Y, and Z axes
- Angular velocity along X, Y, and Z axes

The resultant magnitudes are computed as:

- Acceleration magnitude  
  |a| = √(ax² + ay² + az²)

- Angular velocity magnitude  
  |ω| = √(ωx² + ωy² + ωz²)

These values are continuously monitored by the Arduino.

---

### Multi-Stage Fall Detection Algorithm
A fall is detected only when a sequence of physical conditions occurs in order. This multi-stage logic significantly reduces false detections.

#### Stage 1: Freefall Detection
A fall typically begins with a brief freefall phase where acceleration drops below gravitational acceleration.

Condition:
- |a| < predefined freefall threshold

If detected, the system enters a potential fall state.

---

#### Stage 2: Impact Detection
After freefall, impact with the ground produces a sharp acceleration spike.

Condition:
- |a| > predefined impact threshold

This confirms that a strong collision has occurred.

---

#### Stage 3: Post-Fall Inactivity
After impact, a real fall is usually followed by reduced movement.

Conditions:
- Acceleration remains near 1g
- Angular velocity remains below a defined threshold
- These conditions persist for a short duration

Only if inactivity is observed is the fall confirmed.

---

### Fall Confirmation
A fall is confirmed only when:
- Freefall is detected  
- Followed by impact  
- Followed by post-fall inactivity  

Once confirmed, the system reports the fall event through serial output. No LED or hardware indicator on pin 13 is used in this implementation.

---

## Hardware Components
- Arduino UNO or compatible board
- MPU6050 accelerometer and gyroscope module
- Connecting wires
- USB or external power supply

---

## Software Tools
- Arduino IDE
- Embedded C (Arduino language)
- Wire library
- MPU6050 sensor library
- Serial Monitor for real-time observation


