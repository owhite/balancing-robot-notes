# Pendulum State-space Model Generation and Validation

## 🧠 Workflow Summary
```plaintext
dump_stls.py CAD Export JSON from Rhino
   ↓
   ├── generates STLs
   └── pendulum_metadata.json (stl files, origin, axis of rotation)
   ↓
$ ./generate_LQR_data.py pendulum_metadata.json
   ↓
   ├── computes mass, CoM, inertia
   ├── pendulum_LQR_data.json
   └── writes LQR + validation JSON
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

---

## Failure of the week:
For a while I tried working on a Python + Trimesh + PyBullet workflow, [do not use](../DOCS/oct11_LQR_modeling.md)

## Notes for eventual tune and test

- Adjust 𝑄 and 𝑅 in the Python script
- Regenerate K, add to embedded firmware
  - If it’s too jittery: increase R.
  - If it’s too sluggish: increase the top-left entry in Q.

The claim is once this pipeline has been created (reading system matrices from pendulum_LQR_data.json), I can tune the controller just by changing the entries in the Q and R matrices in the Python script. *"No guesswork. No PID voodoo."* says ChatG. 


