## Oct 11, 2025
# Pendulum Model Validation – Continuation Session Prompt

## Context Recap

We are working on a Python + Trimesh + PyBullet workflow to:
- Calculate the **total mass**, **center of mass**, and **moment of inertia** for a pendulum assembly.
- Export a **URDF** with correct joint reference alignment.
- Generate a **JSON file** with all derived quantities for **LQR control** and **physics verification**.

### Current Program Outputs

From CAD → JSON input, the script now produces:
- ✅ `pendulum_assembly.urdf`  
  - Contains the correctly referenced joint pivot and adjusted visual origins.
- ✅ `pendulum_LQR_data.json`  
  - Contains:

| Variable | Description |
|-----------|-------------|
| `mass_kg` | Total pendulum mass |
| `moment_of_inertia_kg_m2` | Scalar moment of inertia about rotation axis |
| `r_m` | Pivot-to-CoM distance |
| `mgr_over_I` | Gravitational stiffness term |
| `omega_n_rad_per_s` | Natural angular frequency |
| `expected_period_s` | Small-angle oscillation period |
| `A_matrix`, `B_matrix` | Linearized state-space model |

---

## Verification Focus for Next Session

We will **verify and challenge the model’s accuracy**.

### 1. Analytical Validation
- Confirm \( I \approx m r^2 \) is within a reasonable range.
- Recalculate \( \omega_n = \sqrt{mgr/I} \) manually from JSON and verify consistency.
- Perform unit sanity checks.

### 2. Simulation Validation
- Load the generated URDF into PyBullet.
- Apply a small initial angle and simulate free fall.
- Measure the oscillation period and compare to theoretical \( T = 2\pi/\omega_n \).
- Plot or print the ratio \( T_{sim} / T_{theory} \).

### 3. Error Analysis
- Identify whether errors originate from:
  - STL alignment (pivot mismatch)
  - Density inaccuracies
  - Incorrect unit scaling
  - Numerical instability in inertia computation

### 4. Deliverables for Verification
- Python script to simulate free oscillation and extract period.
- Comparison table between theoretical and simulated values.
- Optional visual overlay in PyBullet showing pivot, CoM, and rotation axis.

---

## Goals for Next Session
- Build confidence that the pendulum’s LQR model matches the geometric and dynamic reality.
- Quantify model error in both frequency and amplitude.
- Finalize a reliable mechanical model ready for control testing.

---
# 🧩 Pendulum Project Program Inventory

## 1. Geometry & Input Stage
| Program/File | Purpose | Notes |
|---------------|----------|-------|
| **CAD Program** (e.g. dump_stls.py) | Designs pendulum components and assembly | Exports `.stl` files in **global coordinates** (critical for alignment) |
| **Exported STL files** | Geometry for each part (`housing.stl`, `steel.stl`, etc.) | Units in **mm**, placed correctly in global CAD frame |
| **CAD → JSON exporter** | Produces metadata describing:  <br>• `file_path` <br>• `meshes` (filenames & densities) <br>• `axis_of_rotation` <br>• `origin` | This JSON is the *starting input* to the analysis pipeline |

Example input JSON:
```json
{
  "file_path": "/Users/owhite/Downloads/",
  "meshes": {
    "part1.stl": { "density": 2700.0 },
    "part2.stl": { "density": 7850.0 }
  },
  "axis_of_rotation": [0.0, 1.0, 0.0],
  "origin": [0.0, 0.0, 400.0]
}
```

---

## 2. Physical Property Calculation
| Program | Filename | Purpose |
|----------|-----------|----------|
| **Inertia and URDF Generator** | `pybullet_generator.py` | Main script that: <br>• Loads STL meshes using **Trimesh** <br>• Calculates total mass, CoM, and inertia <br>• Generates `pendulum_assembly.urdf` and `pendulum_LQR_data.json` |
| **Dependencies** | `trimesh`, `numpy`, `json`, `os`, `xml.etree.ElementTree`, `minidom` | No custom external data beyond STL + JSON input |

### Outputs
1. ✅ **URDF file** (`pendulum_assembly.urdf`) — used by PyBullet  
2. ✅ **JSON file** (`pendulum_LQR_data.json`) — for LQR and model validation  

---

## 3. Simulation and Visualization
| Program | Filename | Purpose |
|----------|-----------|----------|
| **PyBullet Visualizer** | `display_urdf.py` | Opens the URDF in **PyBullet GUI**, renders ground plane, origin axes, joint axis, and verifies correct pivot position |
| **PyBullet Environment** | Installed in Conda env (`pybullet-env`) | Used for 3D physics simulation, free-swing test, and debugging URDF alignment |
| **plane.urdf** | Included from `pybullet_data` | Ground reference to visualize z=0 plane alignment |

### Visualization Features
- Draws red/green/blue **world axes**
- Draws **yellow line** for the pendulum joint axis
- Marks the **pivot** with a small sphere
- Prints joint origin and axis vectors for debug alignment

---

## 4. Verification & Analysis Stage
| Program | Purpose |
|----------|----------|
| **LQR JSON Analyzer** | Reads `pendulum_LQR_data.json` <br>Computes A/B matrices, `mgr_over_I`, `ωₙ`, `T` | Confirms small-angle dynamics consistency |
| **Simulation Verification Script** *(next session)* | Will perform a free-swing test in PyBullet | Compares simulated period `T_sim` to theoretical `T_expected` |

### Expected Outputs
- Theoretical frequency and period
- PyBullet-measured oscillation data
- Ratio `T_sim / T_expected`
- Validation plots or printed results

---

## 5. Environment Setup
| Tool | Purpose |
|------|----------|
| **Conda Environment** (`pybullet-env`) | Isolated Python environment containing:  <br>`pybullet`, `trimesh`, `numpy`, `matplotlib` |
| **Command examples** |  |
| Create: `conda create -n pybullet-env python=3.11`  |
| Activate: `conda activate pybullet-env`  |
| Install: `pip install pybullet trimesh numpy matplotlib`  |

---

## 🧠 Workflow Summary
```plaintext
CAD (STLs) 
   ↓
dump_stls.py CAD Export JSON (file paths, densities, axis, origin)
   ↓
pybullet_generator.py
   ├── computes mass, CoM, inertia
   ├── generates URDF (adjusted for joint reference)
   └── writes LQR + validation JSON
   ↓
display_urdf.py (PyBullet)
   ├── loads URDF
   ├── visualizes pivot and CoM alignment
   └── prepares for dynamic validation
   ↓
Next: simulate free swing → extract period → verify ωₙ consistency
```
---

## Goals for Next Session
- Build confidence that the pendulum’s LQR model matches the geometric and dynamic reality.
- Quantify model error in both frequency and amplitude.
- Finalize a reliable mechanical model ready for control testing.

---
## DEETS
- git hash ID: 57efa223f15b6a0a454531ebcfd7cc58b7a34232
