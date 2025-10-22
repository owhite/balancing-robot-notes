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
$ ./LQI_experiment.py -p /dev/cu.usbmodem178888901 -j pulse.json
```

json command passed to teensy:
``` {'cmd': 'send', 'pulse_torque': 0.4, 'total_us': 2000000, 'pulse_us': 500000}```

This spins the **bare** motor (no pendululum and gives this result: 

<img src="Figure_1.png" alt="Plot result" width="300"/>

Estimated decay constant 
- `λ = 3.1526 s⁻¹` 
- `b=3.15e-04`

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

Once 𝑄, 𝑅 are computed, this enables you to move variables on to the teensy for:  
**𝐾 = [𝐾𝜃, 𝐾𝜃˙,𝐾𝑖]**

## Next Steps

I am attaching a program that will send values to my teensy and perform graphing

The inputs are passed as json on parameters, and will include:
Kt = 0.005617 Nm, λ = 3.1526 s⁻¹ , b=3.15e-04 and Ts = 0.002

they will be passed in this way: 

{
  "qterm": "[100.0, 1.0, 500.0]",
  "rterm": 1.0,
  "Kt": 0.005617,
  "lambda": 3.1526,
  "Ts": 0.002,
  "b_decay": 0.000315,
  "torque": 0.2,
  "theta": 3.14,
  "total_ms": 3000000,
  "LQI_path": "/Users/owhite/MESC_brain_board/teensy40/LQI_experiment",
  "Q": "[1.0, 1.0, 1.0]",
  "R": 1.0
}

And end up in params, for example:

params["Kt"] = 0.005617
params["lambda"] = 3.1526
params["Ts"] = 0.002
params["b_decay"] = 0.000315

So the user can change some variables, for example, qterm, rterm, torque, theta, total_ms

Your job is this. At the code comment: # COMPUTE cont2discrete() HERE

create matrices for A_c, B_c and calculate the K gains using cont2discrete() 

<img src="Figure4.png" alt="Plot result" width="300"/>

