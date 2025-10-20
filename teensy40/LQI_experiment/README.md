# LQI

This document describes the workflow for running LQI

---

## Goals

- Apply known torque inputs (pulses) to motor system.  
- Measure **position** and **velocity** response over time.  
- Estimating the Decay Constant (λ) and Damping Coefficient (b)
- 

---

## Motor Constants

After applying a known torque pulse to the motor, the control program records the motor’s angular velocity as it freely spins down once the torque command is set to zero. During this free-decay phase, no active control or regenerative braking is applied—the motion is governed solely by passive damping effects such as bearing friction, air drag, and internal electrical losses.

The motor’s deceleration follows the first-order dynamic model:

𝐽𝜔˙ + 𝑏 = 0

whose analytical solution is:

𝜔(𝑡) = 𝜔0𝑒 <sup>−(𝑏/𝐽)𝑡</sup>

Taking the natural logarithm of velocity gives a linear relationship:

ln(𝜔) = ln(𝜔0) − 𝜆𝑡

where the slope 𝜆 = 𝑏 / 𝐽 is the decay constant.

The program automatically identifies the point where torque drops to zero and fits a straight line to the logarithm of the velocity data in the decay region. From this, it computes:

- Decay constant: 𝜆 = 𝑏 / 𝐽
- Damping coefficient: 𝑏 = 𝜆 𝐽 (if the inertia is known or estimated)

These values are displayed in the GUI label and provide a direct, repeatable way to quantify mechanical damping in the system. The estimated damping coefficient 𝑏 is used later in the control model to refine LQR/LQI design and improve accuracy in simulated or analytical predictions.

## Logging

Each sample records:

- `t_us` (µs since start of experiment)  
- `torque` (normalized ESC command)  
- `pos` (measured rotor angle, rad)  
- `vel` (measured rotor velocity, rad/s)  

These logs are consumed later in Python for least squares fitting.

<img src="Figure_1.png" alt="Plot result" width="300"/>

---

## Python Interface

We will use a Python + matplotlib GUI to:

1. Accept user parameters:
   - `Kt` (Nm/A)  
   - `pulse_duration` (ms)  
   - `torque_request` (Nm)  
2. Compute:
   - Phase current: \(I = T / K_t\)  
   - Normalized command: \(I / I_\text{max}\)  
3. Send configuration (`user_pulse_torque`, `user_pulse_us`, `user_total_us`) to the Teensy over serial.  
4. Run the experiment.  
5. Retrieve JSON logs and overlay experimental response with PyBullet predictions.  

Launch the graphing program
```
$ ./torque_raise.py  /dev/cu.usbmodem178888901
```

---

## Modeling & Validation

- In PyBullet (or analytical model), simulate the pendulum response to the same pulse torque input.  
- Compare **position** and **velocity** traces to experimental data.  
- Apply **least squares fitting** to minimize error and update estimated parameters (inertia, damping, Kt).  
- Iterate until model matches physical system.  

---

## Next Steps

- Implement the Python ↔ Teensy serial protocol.  
- Run a baseline 0.2 Nm, 85 ms pulse and verify response.  
- Scale experiments:
  - Vary pulse amplitudes (0.1 Nm, 0.3 Nm, …).  
  - Vary durations (50 ms, 100 ms, …).  
- Use PRBS excitation once pulse responses look consistent.  
- Validate the fitted model with new inputs not used for training.  
