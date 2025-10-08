# Prompt for Future Use

I have a physical model of a pendulum system with detailed knowledge of the components — including volumes, densities, dimensions, centers of mass, and moments of inertia. The pendulum is already modeled in PyBullet, so I can compute torques, angles, and velocities accurately.

## I would like to:
- Derive the state-space equations from first principles using Newton’s laws — considering rotational inertia, damping, torque balance, and (if applicable) stiffness or restoring torque.
- Convert these equations into state-space form, showing the symbolic steps from differential equations to the matrices 
A, B, C, and D.
- Plug in my estimated physical parameters (J,b,k) to generate numerical A,B,C,D matrices suitable for use in an LQR controller.
- Refine and tune the physical model later using experimental data (e.g., PRBS excitation, step tests, or free oscillations) to match real system dynamics.

Provide both the symbolic derivation and the numerical implementation (in Python, using numpy and scipy.signal.StateSpace).

Finally, explain how the LQR gains (K) would be computed and interpreted in the context of the pendulum’s physics.

## PRBS
So far I have tried running a series of PRBS (Pseudo-Random Binary Sequence) experiments to estimate a transfer function between motor torque and pendulum velocity or angle. The goal was to fit a low-order model, extract physical parameters (J,b,k), and then design an LQR controller.

However, we found that the experimental identification method (PRBS-based model fitting) produced unstable or inconsistent estimates — leading to misleading transfer functions and negligible difference between open-loop and closed-loop responses.

Main Issues We Encountered with PRBS

## 1. Limited Frequency Range
- The bit_time_ms = 30 setting means each torque command lasts 30 ms.
- That limits the Nyquist frequency (the highest resolvable excitation frequency) to about 16.7 Hz.
- Your pendulum’s natural frequencies (where it actually “rings”) are around 40–50 Hz.
- The PRBS signal therefore never excites those key frequencies.
- As a result, the system identification is under-determined — it tries to estimate stiffness k and inertia J from data that doesn’t contain enough information about them.

## 2. Noise and Short Duration
- A 5-second PRBS burst doesn’t give enough averaging for low-frequency accuracy.
- The frequency resolution is coarse, and any sensor noise or delay has a large impact on the identified model.
- This leads to high-variance fits, where parameters jump around between runs.

## 3. Model Overfitting / Unrealistic Dynamics
- The fitted transfer functions sometimes had unstable poles or absurdly high gains.
- These models led to step responses that were unphysical (e.g., enormous or diverging).
- Even when stable, the resulting LQR controller barely shifted the system poles, meaning the controller had no effective authority.

## What We Concluded
- The low-frequency damping (viscous drag, current→velocity slope below ~10–15 Hz) is the only reliably measurable part of the PRBS data right now.
- The torque→angle DC gain is also measurable but small, confirming the system is quite stiff.

- To get J and k accurately, you either need:
  - A faster PRBS (bit_time_ms = 5–10) and a longer test (10–20 s), or
  - A direct quasi-static test — apply constant torque, measure steady-state angle, and compute k=τ/θ.

## Recommended Path Forward
- Start from physics — build your analytical model:
  - Derive 𝐽, 𝑏, 𝑘 from mass, geometry, and material properties.
  - Construct the symbolic state-space equations.
- Implement LQR using those physically derived A,B,C,D matrices.
- Simulate the controller in PyBullet.
- Run small PRBS or step tests later to refine b and confirm system resonance frequency.

This hybrid approach — physical modeling first, data refinement later — will give you a stable foundation for designing your controller without depending entirely on noisy PRBS identification.

## New terms

**pole** is a mathematical point that describes how your system behaves over time — basically, how it responds to changes.

- Each pole corresponds to one natural behavior — like an oscillation, decay, or instability.
- Where the poles sit in the complex plane tells you how stable and how fast your system is.
- Poles farther left → faster, more stable response.
- Poles closer to zero or positive → slower, or unstable.

Why “pole shift” matters: when you apply feedback control (like LQR), the controller moves those poles — literally shifting them in the complex plane.

## Channel?

In control systems, a **channel** refers to a specific **input–output relationship** within a system — essentially, the *path* by which one variable affects another.

In a multi-variable system (like a pendulum with torque, velocity, and angle), you can study several different channels, each telling you something about the physics of the system.

| Channel | Input | Output | Physical Meaning |
|:--|:--|:--|:--|
| **Torque → Velocity** | Commanded torque (motor current) | Angular velocity | Describes **damping behavior** — how torque accelerates or slows motion. |
| **Torque → Angle** | Commanded torque | Angular position | Describes **stiffness and inertia** — how torque affects steady-state deflection. |

---

## Why Channels Matter

Each channel provides insight into a different dynamic property of the system:

- The **torque→velocity** channel is **dominated by damping** and usually looks like a **first-order system**.  
  It’s responsive and easy to measure accurately, even in short experiments.

- The **torque→angle** channel is **stiffer and slower**, so it needs more time or energy to show its behavior.  
  When your PRBS (pseudo-random binary sequence) excitation is too short or too low in frequency, the angle barely moves — making that channel noisy and unreliable for identifying stiffness \(k\) and inertia \(J\).

---

## How This Applies to Your Pendulum

When running PRBS tests on the pendulum:

- **Torque→Velocity channel:** gives reliable data about **damping (b)**.  
  You can use this to tune your viscous damping coefficient and model energy losses.

- **Torque→Angle channel:** gives weak or noisy data about **stiffness (k)** and **inertia (J)**.  
  This is because your pendulum is quite stiff, and the PRBS doesn’t excite its natural frequency range strongly enough.

> **Summary:**  
> The velocity channel is trustworthy for low-frequency dynamics (10–15 Hz), while the angle channel is too stiff and under-excited under current test conditions.

---

## Key Takeaways

| Concept | Meaning |
|:--|:--|
| **Channel** | A specific input–output relationship (e.g., torque→velocity or torque→angle). |
| **Torque→Velocity** | Measures how torque changes rotational speed — dominated by damping. |
| **Torque→Angle** | Measures how torque changes position — dominated by stiffness and inertia. |

**Practical Impact** Focus on the torque→velocity channel for model fitting and damping estimation; refine torque→angle later using quasi-static or longer tests.

