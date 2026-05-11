# Intergration-of-Eddy-Current-Braking-System-with-Frictional-Braking-System
A ESP32-based system that measures real-time wheel speed (RPM/kmph) via a Hall effect sensor and calculates braking distance when the brake is applied. A dual NEMA17 stepper motor actuator (driven by TB6600) physically extends/retracts based on speed and brake state.

# Team Members
From Department of Automobile and Mechanical Engineering , Thapathali campus, IOE
* **Anish Bimali** (THA078BAM003)
* **Mahesh Giri**(THA078BAM029)
* **Shishir Khanal**(THA078BAM038)
* **Sujal Dahal**(THA078BAM040)

# Hardware specifications
* **MCU - ESP32**
* **Speed Sensor- Hall effect sensor** (8 magnets) — GPIO 23
* **Brake Sensor- Hall effect sensor** — GPIO 35
* **Stepper DriverTB6600** — PUL: GPIO 26, DIR: GPIO 27, ENA: GPIO 25
* **Motors2× NEMA17** wired in parallel (set TB6600 = combined current)
* **Display16×2 I2C LCD (0x27)**
* **Wheel Radius 0.2032 m**

# Features:
* **RPM & Speed — Hall pulse counting with debounce, noise rejection, and exponential smoothing**
* **Braking Distance — State machine (IDLE → MEASURING → HOLDING) integrates speed over time; result held on LCD/Serial for 10 s**
* **Stepper Actuator — Extends 2 mm at ≥15 km/h + brake pressed; retracts otherwise (AccelStepper smooth ramping)**
* **16×2 LCD — Live RPM, speed, brake status, actuator state, and braking distance**
