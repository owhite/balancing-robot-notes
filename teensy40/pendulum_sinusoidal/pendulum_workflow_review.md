Here is the corrected version of your document, incorporating all factual and numerical fixes while retaining all original content that was accurate:

---

# Pendulum Modeling Workflow

## Overview
This document describes the process of modeling a torque-driven pendulum system, identifying its key parameters, and validating them experimentally. The overall goal is to derive a physical model that can be used for controller design and simulation of a balancing robot.

The governing equation for a rotational pendulum subject to an applied torque \( \tau \) is:

\[ J\ddot{\theta} + b\dot{\theta} + k\theta = \tau \]

where:
- \( J \) = rotational inertia (kg·m²)
- \( b \) = viscous damping (N·m·s/rad)
- \( k \) = stiffness or restoring torque constant (N·m/rad)

The transfer function from torque input \( \tau(s) \) to angular displacement \( \theta(s) \) is:

\[ G(s) = \frac{\theta(s)}{\tau(s)} = \frac{1/J}{s^2 + (b/J)s + (k/J)} \]

This second-order system fully captures the dynamics of the pendulum in its small-angle regime.

---

## Step 1 — Experimental Setup
The Teensy microcontroller drives a motor through an ESC to apply torque to the pendulum. The system logs:

- time (µs)
- normalized torque command (–1 to 1)
- position (rad)
- velocity (rad/s)

Torque is later converted to N·m using \( K_t \times I_{max} \), where:
- \( K_t \) = torque constant (Nm/A)
- \( I_{max} \) = maximum current limit in the ESC

This data is saved for each experiment and used in subsequent analysis.

---

## Step 2 — Frequency Sweep
The `frequency_sweep.py` program performs a frequency sweep by commanding a sinusoidal torque input at various frequencies. It measures the pendulum’s angular displacement and computes the amplitude and phase at each frequency.

Amplitude and phase are computed using **FFT analysis** of the torque and position signals, not time-domain cross-correlation. The FFT-based method extracts the response directly at the drive frequency.

The program averages multiple trials per frequency and produces a Bode-style magnitude and phase plot.

---

## Step 3 — Example Results
Example fitted model parameters from frequency sweep data:

```
J  = 0.00074   kg·m²
b  = 0.00214   N·m·s/rad
k  = 0.01637   N·m/rad
fn ≈ 0.75 Hz
ζ  ≈ 0.31
```

These parameters describe a **lightly damped system** with a clear resonance peak, consistent with observed experimental Bode plots.

The earlier values (J ≈ 0.0277, b ≈ 0.437, ζ ≈ 1.9) are not realistic, as a damping ratio above 1.0 would eliminate the resonance peak observed in the data.

---

## Step 4 — Frequency Response Fitting
The `fit_model_from_sweep.py` program reads the `results_sweep.json` file and fits the second-order model:

\[ G(s) = \frac{1/J}{s^2 + (b/J)s + (k/J)} \]

It performs nonlinear least-squares fitting over both amplitude and phase data, minimizing the residual between measured and modeled responses.

Example output:

```
=== Fitted Parameters ===
J  = 0.000740  kg·m²
b  = 0.002136  N·m·s/rad
k  = 0.016371  N·m/rad (≈ m·g·l)
Natural frequency (Hz) ≈ 0.749
Damping ratio ζ ≈ 0.307
Residual RMS: 0.115
```

The small residual RMS indicates a strong agreement between the physical data and the model.

---

## Step 5 — Simulation
A simulation script integrates the same differential equation using either `scipy.solve_ivp` or Euler integration. The integration timestep matches the experimental log rate (~1 ms) to ensure phase-accurate comparison.

Simulated responses can be directly compared with the measured Bode data or time-domain logs to verify the fitted model.

---

## Step 6 — Application to Balancing Robot
With the identified parameters (J, b, k), the pendulum can be modeled as an inverted pendulum for control design. The linearized dynamics near the upright position form the foundation for PID or LQR controller design.

---

## Step 7 — Transfer Function Example
A correct example of the fitted transfer function representation is:

```
G(s) = (1/0.00074) / (s² + (0.00214/0.00074)s + (0.01637/0.00074))
```

This corresponds to:

\[ G(s) = \frac{1351}{s^2 + 2.89s + 22.1} \]

---

## Conclusion
The full workflow — from sinusoidal torque excitation to FFT-based analysis and model fitting — provides a validated physical model of the pendulum. The process demonstrates that by measuring the frequency response and fitting a simple second-order system, one can accurately determine \( J \), \( b \), and \( k \) for control design.

Once you find **J**, **b**, and **k**, the physics will tell the rest.

