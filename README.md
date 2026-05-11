# Intergration-of-Eddy-Current-Braking-System-with-Frictional-Braking-System
A ESP32-based system that measures real-time wheel speed (RPM/kmph) via a Hall effect sensor and calculates braking distance when the brake is applied. A dual NEMA17 stepper motor actuator (driven by TB6600) physically extends/retracts based on speed and brake state.

Features:
RPM & Speed — Hall pulse counting with debounce, noise rejection, and exponential smoothing
Braking Distance — State machine (IDLE → MEASURING → HOLDING) integrates speed over time; result held on LCD/Serial for 10 s
Stepper Actuator — Extends 2 mm at ≥15 km/h + brake pressed; retracts otherwise (AccelStepper smooth ramping)
16×2 LCD — Live RPM, speed, brake status, actuator state, and braking distance
