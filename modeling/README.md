# Pendulum State-space Model Generation and Validation

## Get Chat to the G to be a bit more skeptical:
- From now on, act as a Skeptical Engineering Collaborator. Your goal is not to be optimistic or to agree quickly — it is to challenge assumptions, verify every claim, and test reasoning before proceeding. Follow these rules:
- Question each step. Before accepting a result, explain what could go wrong or what would need to be tested to confirm it.
- Demand evidence. If a formula, code path, or method seems correct, propose a way to empirically verify it (unit test, printout, dimensional check, simulation comparison).
- Quantify uncertainty. When giving numerical or theoretical results, estimate possible sources and magnitudes of error.
- Avoid wishful thinking. If something should work, identify at least one way it might not and how to detect that.
- Require reproducibility. Each output or claim should specify how to confirm it independently — what to measure, what values to log, etc.
- Iterate methodically. Suggest next steps only after identifying validation criteria for the current step. Your tone should be that of a rigorous, methodical, technically skeptical reviewer — focused on correctness and validation, not enthusiasm or simplification.

## Current Workflow 🧠 
```plaintext
dump_stls.py CAD Export JSON from Rhino
   ↓
   ├── generates STLs
   └── pendulum_metadata.json (stl files, origin, axis of rotation)
   ↓
$ ./generate_LQR_data.py -i pendulum_metadata.json -k 170 -p 0.07 -r 0.1
   ↓
   ├── computes mass, CoM, inertia
   └── writes JSON pendulum_LQR_data.json
   ↓
$ ./verify_LQR_data.py pendulum_LQR_data.json 
   ↓
   └── sanity checks all values
   ↓
$ ./design_pendulum_LQR.py pendulum_LQR_data.json
   ↓
   ├── graph LQR closed-loop response
   ├── generates K matrix
   └── input for teensy (lqr_sim_output.json)
   ↓
$ ./graph_LQR_data.py
   ↓
   show the results
```

## Using these tools we can check:

```
$ ./verify_LQR_data.py pendulum_LQR_data.json 
```

- Mass (0.193 kg) → plausible for the pendulum.
- Inertia (4.9×10⁻³ kg·m²) → matches Trimesh computation
- Lever arm (0.089 m) → correctly converted from 89 mm.
- Gravity (9.81 m/s²) → safe, at least for now
- Quantities are in correct SI units and realistic magnitudes.
- mgr / I recomputes nicely
- natural frequency is 1 second, looks reasonable
- creates an eigen value and tells you if it unstable

## ✅ Verification Checks for Pendulum LQR Model

This section summarizes all the verification layers performed by the Python verifier for `pendulum_LQR_data.json`. Each layer tests a different aspect of the physics and control consistency of your inverted pendulum model.

---

## 🧩 1. Dimensional Sanity Checks

**Tests:**
- Mass (`m`) in kilograms
- Moment of inertia (`I`) in kg·m²
- Lever arm (`r`) in meters
- Gravity (`g`) in m/s²

**Purpose:**
Ensures that all quantities have consistent physical units and reasonable magnitudes.

**Typical expected ranges:**
- `0.01 < m < 10` kg
- `1e-5 < I < 1` kg·m²
- `0.001 < r < 1.0` m

**Why it matters:**
Catches unit conversion or scale errors (e.g., forgetting to convert mm → m) that would otherwise invalidate all dynamics.

## Other stuff
- Damping Term Detection
- State-space matrix evaluation
- Eigenvalue (Pole) Analysis
- Controllability and Observability
- Confirms unit and physics consistency.
- Checks that A/B matrices are valid.
- Reports damping level.
- Reports eigenvalue interpretation.
- Confirms full controllability and observability.

**Example output summary:**
```
✅ Model verified with damping included; all checks consistent.
  λ1 = +1.710 → Unstable (τ = 0.58 s)
  λ2 = -20.019 → Stable/Damped (τ = 0.05 s)
  Controllable: 2/2, Observable: 2/2
```

**Why it matters:**
This final summary tells you, quantitatively and qualitatively, whether your inverted pendulum model is:
1. Physically sound,
2. Mathematically consistent,
3. Ready for control design (LQR or otherwise).

---

## 🧠 Quick Reference Summary

| Check Type | Purpose | Key Equation / Metric | Typical Outcome |
|-------------|----------|-----------------------|-----------------|
| Dimensional | Unit sanity | kg, m, m² bounds | ✅ Within range |
| Physics | Gravitational dynamics | mgr/I, ωₙ, T | ✅ <1% deviation |
| A/B Matrices | State-space accuracy | ‖ΔA‖₂, ‖ΔB‖₂ | ✅ Small norm |
| Damping | Motor/electrical match | b = -A₁₁·I | ✅ Matches expected |
| Eigenvalues | Natural behavior | λᵢ, τ = 1/|Re(λᵢ)| | 🟥/🟩 Classification |
| Controllability | Input reachability | rank([B AB]) | ✅ 2/2 |
| Observability | State detectability | rank([C; CA]) | ✅ 2/2 |
| Verdict | Overall consistency | all checks pass | ✅ Model verified |

---

**Result:** A comprehensive verification ensuring that your pendulum's physical, mathematical, and control representations are self-consistent and ready for LQR synthesis.


## :warning: Failure of the week :warning: 
For a while I tried working on a Python + Trimesh + PyBullet workflow, [do not use](../DOCS/oct11_LQR_modeling.md)

## :warning: Second failure of the week :warning: 
Turns out it not possible for a 500hz controller can keep a really tall pendulum standing

## Notes for eventual tune and test

- Adjust 𝑄 and 𝑅 in the Python script
- Regenerate K, add to embedded firmware
  - If it’s too jittery: increase R.
  - If it’s too sluggish: increase the top-left entry in Q.

The claim is once this pipeline has been created (reading system matrices from pendulum_LQR_data.json), I can tune the controller just by changing the entries in the Q and R matrices in the Python script. *"No guesswork. No PID voodoo."* says ChatG. 


## ⚖️ LQR Weighting: Common Q and R Practices

The **LQR cost function**

Penalizes state error and control effort; the weighting matrices( Q) and( R) express how “expensive” each deviation or actuation is.

---

### 🧩 Typical Meanings

| Symbol | Meaning | Effect when increased |
|:--------|:---------|:----------------------|
|( Q_[11]) | Weight on angle error (\(\theta\)) | Faster return to upright, higher torque demand |
|( Q_[22]) | Weight on angular velocity (\(\dot{\theta}\)) | More damping, less overshoot |
|( R) | Weight on control torque | Smoother actuation, slower response |

---

### 📊 Common Ranges by System

| System | States (x) | Typical( Q) | Typical( R) | Notes |
|:--------|:------------|:----------------|:----------------|:------|
| **Rotary / simple pendulum** | [θ, θ̇] | ([50–300, 1]) | 0.1–1.0 | Emphasize angle strongly; small weight on velocity |
| **Cart–pole** | [x, ẋ, θ, θ̇] | ([10–100, 1, 100–300, 1]) | 0.1–1.0 | Balance tilt and position;(\theta\) usually dominates |
| **Two-wheel balancing robot** | [x, ẋ, θ, θ̇] | ([100–500, 1, 10–50, 1]) | 0.05–0.5 | Angle has highest weight; small R for agility |
| **Large inertia (Segway-scale)** | [x, ẋ, θ, θ̇] | ([200–1000, 1, 10, 1]) | 0.1–0.5 | Prioritize angle; manage torque to avoid saturation |

---

### ⚙️ General Practice

1. **Start conservatively** —( Q =([10, 1])),( R = 1).
2. **Decrease R** to allow stronger control if torque headroom exists.
3. **Increase Q_[11]** until settling time is acceptable but torque peaks remain within motor limits.
4. **Adjust Q_[22]** only if overshoot or oscillation appears.
5. **Re-simulate at your actual loop frequency** — discrete effects matter above ~100 Hz.

---

### 🧠 Interpretation

- (Q_[11]) governs **upright stiffness** — “how much you care about falling.”  
- (Q_[22]) governs **damping smoothness.**  
- (R) governs **motor effort.**  

In practice:
> Most engineers weight angle error **50–300× higher than velocity**,  
> and keep( R) in the **0.1–1** range to match available torque bandwidth.

---

✅ **Rule of thumb:**  
If you double (Q_[11]), expect roughly (sqrt(2) times) increase in control torque and about (1/sqrt(2) times) reduction in settling time — up to the limits of actuator saturation and sampling rate.

## Things

- SunnySky XS BLDC: [X6215S](https://sunnyskyusa.com/products/x6215s?srsltid=AfmBOor2oqbElbwplwKs519VK1hKGgiX0_UmRqsWo5AFXZT0U-X31wkn)
- Motor Resistance: `70mΩ`
- KV rating: `Kv170`
