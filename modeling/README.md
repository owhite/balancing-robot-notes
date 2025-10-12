# Pendulum State-space Model Generation and Validation

## Get Chat to the G to be a bit more skeptical:
```
Very good.

From now on, act as a Skeptical Engineering Collaborator. Your goal is not to be optimistic or to agree quickly — it is to challenge assumptions, verify every claim, and test reasoning before proceeding. Follow these rules:

Question each step. Before accepting a result, explain what could go wrong or what would need to be tested to confirm it.

Demand evidence. If a formula, code path, or method seems correct, propose a way to empirically verify it (unit test, printout, dimensional check, simulation comparison).

Quantify uncertainty. When giving numerical or theoretical results, estimate possible sources and magnitudes of error.

Avoid wishful thinking. If something should work, identify at least one way it might not and how to detect that.

Require reproducibility. Each output or claim should specify how to confirm it independently — what to measure, what values to log, etc.

Iterate methodically. Suggest next steps only after identifying validation criteria for the current step. Your tone should be that of a rigorous, methodical, technically skeptical reviewer — focused on correctness and validation, not enthusiasm or simplification.
```

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
Here we are

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

---

## ⚖️ 2. Physics Consistency Checks

**Formulas checked:**
\[ \frac{mgr}{I}, \quad \omega_n = \sqrt{\frac{mgr}{I}}, \quad T = \frac{2\pi}{\omega_n} \]

**Tests:**
- Recomputes \( mgr/I \) from the stored values.
- Compares computed vs. stored \( \omega_n \) and \( T \).
- Flags differences greater than 0.1%.

**Why it matters:**
Confirms the physical core of the pendulum model — that gravitational torque and inertia are balanced correctly. A mismatch means incorrect geometry or density.

---

## 🧮 3. State-Space Matrix Verification

**Expected matrices:**
\[
A = \begin{bmatrix} 0 & 1 \\ mgr/I & 0 \end{bmatrix}, \quad
B = \begin{bmatrix} 0 \\ 1/I \end{bmatrix}
\]

**Tests:**
- Compares the stored `A` and `B` matrices to the theoretical ones.
- Computes norms \( \|\Delta A\|_2 \) and \( \|\Delta B\|_2 \).

**Why it matters:**
Ensures that your exported state-space model matches the physical small-angle equations. Detects sign errors, damping additions, or incorrect linearization.

---

## ⚙️ 4. Damping Term Detection

**Formula:**
\[ b = -A[1,1] \cdot I \]

**Tests:**
- Detects whether a damping coefficient exists.
- Estimates effective viscous damping \( b \) in N·m·s/rad.
- Optionally compares against `motor_params["b_Nm_s_per_rad"]` if present.

**Why it matters:**
Validates inclusion of motor back-EMF damping and ensures electrical and mechanical models agree. Prevents artificial stability or unrealistic friction.

---

## 📈 5. Eigenvalue (Pole) Analysis

**Computation:**
\[ \lambda_i = \text{eig}(A) \]

**Tests:**
- Lists all eigenvalues of `A`.
- Classifies each pole:
  - 🟥 Positive real → **Unstable** (growth)
  - 🟩 Negative real → **Stable/Damped** (decay)
  - 🟢 Complex → **Oscillatory**
- Computes time constants \( \tau = 1/|\Re(\lambda)| \).

**Why it matters:**
Reveals natural growth and decay rates of your physical pendulum before control. The unstable pole represents the tendency to fall over; the stable pole represents internal damping.

---

## 🔌 6. Controllability and Observability

**Formulas:**
\[
C_o = [B, AB], \quad O_b = \begin{bmatrix} C \\ CA \end{bmatrix}
\]

**Tests:**
- Computes ranks of controllability and observability matrices.
- For a 2-state pendulum: full rank (2/2) expected.

**Why it matters:**
Guarantees that every state (angle, angular velocity) can be influenced by the control torque and observed from sensors. If not full rank, the LQR cannot stabilize the system.

---

## 🧭 7. Stability Summary and Verdict

**Combines all previous results:**
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

## Notes for eventual tune and test

- Adjust 𝑄 and 𝑅 in the Python script
- Regenerate K, add to embedded firmware
  - If it’s too jittery: increase R.
  - If it’s too sluggish: increase the top-left entry in Q.

The claim is once this pipeline has been created (reading system matrices from pendulum_LQR_data.json), I can tune the controller just by changing the entries in the Q and R matrices in the Python script. *"No guesswork. No PID voodoo."* says ChatG. 


## Things

- SunnySky XS BLDC: [X6215S](https://sunnyskyusa.com/products/x6215s?srsltid=AfmBOor2oqbElbwplwKs519VK1hKGgiX0_UmRqsWo5AFXZT0U-X31wkn)
- Motor Resistance: `70mΩ`
- KV rating: `Kv170`
