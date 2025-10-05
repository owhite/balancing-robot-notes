#!/usr/bin/env python3
"""
export_pendulum_model.py

Step 3 of the pendulum identification workflow.
Reads fitted parameters (from rotational_i_viscous.json),
prints derived model details, and exports a clean
pendulum_model.json file for simulation and control design.
"""

import json
import numpy as np

INPUT_FILE = "rotational_i_viscous.json"
OUTPUT_FILE = "pendulum_model.json"

# --- Load fitted parameters ---
with open(INPUT_FILE, "r") as f:
    params = json.load(f)

J = params["J"]
b = params["b"]
k = params["k"]

# --- Derived quantities ---
wn = np.sqrt(k / J) / (2 * np.pi)      # natural frequency (Hz)
zeta = b / (2 * np.sqrt(J * k))        # damping ratio

# --- Print summary ---
print("\n=== Derived Pendulum Model ===")
print(f"J = {J:.6f}  kg·m²")
print(f"b = {b:.6f}  N·m·s/rad")
print(f"k = {k:.6f}  N·m/rad (≈ m·g·l)")
print(f"Natural frequency (Hz) ≈ {wn:.3f}")
print(f"Damping ratio ζ ≈ {zeta:.3f}")

# --- Transfer function (display only) ---
print("\nOpen-loop transfer function:")
print(f"G(s) = 1 / ({J:.6f}·s² + {b:.6f}·s + {k:.6f})")

# --- Write model file ---
model = {
    "J": J,
    "b": b,
    "k": k,
    "natural_frequency_hz": wn,
    "damping_ratio": zeta
}

with open(OUTPUT_FILE, "w") as f:
    json.dump(model, f, indent=2)

print(f"\n✅ Model exported to '{OUTPUT_FILE}'")

