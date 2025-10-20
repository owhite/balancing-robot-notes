# LQI

This document describes the workflow for running LQI

---

## Goals

- Gather up a bunch of values to model the motor
- Thing1
- Thing2
- Thing3 

---

## Motor Constants Ke, Kt, Kv

<img src="figure0.png" alt="Plot result" width="600"/>

## Motor Constants: motor decay

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

## Python Interface

Launch the graphing program

```
$ ./LQI_experiment.py  /dev/cu.usbmodem178888901
```

json command passed to teensy:
``` {'cmd': 'send', 'pulse_torque': 0.4, 'total_us': 2000000, 'pulse_us': 500000}```

This spins the **bare** motor (no pendululum and gives this result: 

<img src="Figure_1.png" alt="Plot result" width="300"/>

Estimated decay constant `λ = 3.1526 s⁻¹` and `b=3.15e-04`
---

## Modeling 

We have a bunch of variables, let's go:

<img src="Figure3.png" alt="Plot result" width="600"/>

Parameters that need to be chosen

| Parameter             | Description                           | Typical Starting Point                                                      |
| --------------------- | ------------------------------------- | --------------------------------------------------------------------------- |
| (Q)                   | State weighting matrix                | diag([q₁, q₂, qᵢ]) — emphasize position & integral more than velocity       |
| (R)                   | Control effort weight                 | scalar; start around 0.1–10 depending on how aggressive you want torque use |
| sampling period (T_s) | Discretization step (for Teensy loop) | 1–2 ms (≈ 500–1000 Hz outer loop)                                           |
| actuator limits       | torque or current saturation          | ± Kₜ · Iₘₐₓ, e.g. ± 1.68 N·m for 30 A                                       |


## Next Steps

- Implement the Python ↔ Teensy serial protocol.  
- Run a baseline 0.2 Nm, 85 ms pulse and verify response.  
- Scale experiments:
  - Vary pulse amplitudes (0.1 Nm, 0.3 Nm, …).  
  - Vary durations (50 ms, 100 ms, …).  
- Use PRBS excitation once pulse responses look consistent.  
- Validate the fitted model with new inputs not used for training.  
