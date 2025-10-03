## Sept 30, 2025
# Workflow: From Rhino to PyBullet

## 1. Geometry Design in Rhino
- Create the pendulum or robot parts as 3D CAD geometry in **Rhino**.
- Keep geometry clean and simplified (avoid unnecessary detail that would inflate the mesh).

## 2. Export to STL
- Export each part as an **STL file** in Rhino using dump_stls.py
- Use consistent units (e.g., meters) to avoid scale mismatches in PyBullet.
- Check mesh orientation (positive Z usually up in PyBullet).

```
$ ./pybullet_generator.py pendulum_metadata.json 
```

## 3. Load Meshes in PyBullet
- Import STL files into PyBullet via `createCollisionShape` and `createVisualShape`.
- Evaluate meshes with tools like **trimesh** 
- Bundle STL parts into a **URDF** file to define joints, masses, and inertial properties.

A baby modeling example of parts that are in pendulum.3dm:
```
 ./sinusoidal1.py 
```

# Next Steps for Pendulum Validation & Modeling

some initial modeling is [here](https://youtu.be/Mqh3uTG2mss). 

so what am I thinking about now? 

### PRBS torque input
- Use Pseudorandom Binary Sequence torque input to test signal that excites many frequencies in the system.
- Useful for system identification, because it gives enough variation to estimate parameters robustly.

### Least Squares
- Use least squares to compare the physical model vs. the theoretical one.
- Commonly used to fit parameters (like damping or inertia) by minimizing the error between measured output (pendulum angle, velocity) and model predictions.

### Validation plan
- apply a known torque, see how the pendulum moves, and check if the real torque response is “in the ballpark” of the theoretical torque.
- a straightforward sanity check before doing full system ID.

### Summary of your modeling plan
- Step 1: Actuate the pendulum with known torque (via ESC command).
- Step 2: Measure motion (angle, velocity).
- Step 3: Compare observed vs. predicted trajectory.
- Step 4: Later, add more structured inputs (PRBS) and use least squares fitting to refine model parameters.

### Set up experiment
- Connect pendulum to ESC/motor.
- Ensure we can send torque commands reliably.

### Baseline test
- Apply a simple constant torque (via ESC).
- Record pendulum motion (angle, velocity).
- Compare observed motion with theoretical model predictions.

### Structured excitation
- Use step torques, sine sweeps, or PRBS signals for richer input data.
- Collect multiple datasets to capture system behavior across conditions.

### Model fitting
- Apply least squares to minimize error between measured response and model output.
- Estimate parameters (e.g., inertia, damping, stiffness).

### Validation
- Run new experiments with different inputs.
- Check if refined model predicts responses accurately.

### Iteration
- Adjust experimental design (inputs, sampling) as needed.
- Refine model complexity (linear vs nonlinear terms) if discrepancies remain.
