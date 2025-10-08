#!/usr/bin/env python3
"""
compare_frf_nyquist.py

Compare measured vs. modeled complex frequency response (FRF)
for the torque-driven pendulum system.

Uses:
- results_sweep.json  (from frequency_sweep.py)
- rotational_i_viscous.json  (from fit_model_from_sweep.py)

Plots both FRFs on the complex plane (Nyquist plot) and shows magnitude/phase residuals.
"""

import json, numpy as np, matplotlib.pyplot as plt

# --- Load measured frequency response data ---
with open("results_sweep.json") as f:
    data = json.load(f)

freqs = np.array([d["freq_hz"] for d in data])
amp_meas = np.array([d["amp_rad"] for d in data])
phase_meas = np.radians([d["phase_deg"] for d in data])

# Measured complex response G_meas(jω)
G_meas = amp_meas * np.exp(1j * phase_meas)

# --- Load fitted parameters ---
with open("rotational_i_viscous.json") as f:
    params = json.load(f)

J = params["J"]
b = params["b"]
k = params["k"]

# --- Compute modeled complex response ---
w = 2 * np.pi * freqs
G_model = (1/J) / ((-w**2 + 1j*(b/J)*w) + (k/J))

# --- Compute modeled magnitude and phase ---
mag_meas = np.abs(G_meas)
mag_model = np.abs(G_model)

# --- Fit a scalar gain to align model amplitude with measured data ---
scale_factor = np.sum(mag_meas * mag_model) / np.sum(mag_model**2)
G_model_scaled = G_model * scale_factor
mag_model_scaled = np.abs(G_model_scaled)

print(f"\nEstimated amplitude scaling factor = {scale_factor:.3f}")

# --- Compute residuals ---
mag_resid = np.abs(np.abs(G_meas) - np.abs(G_model))
phase_resid = np.degrees(np.unwrap(np.angle(G_meas)))
rms_resid = np.sqrt(np.mean(mag_resid**2))

print("=== FRF Comparison ===")
print(f"RMS magnitude residual: {rms_resid:.4f}")
print(f"Mean phase residual: {np.mean(np.abs(phase_resid)):.2f}°")

# --- Plot Nyquist (complex plane) ---
plt.figure(figsize=(6,6))
plt.plot(G_model.real, G_model.imag, 'r-', label='Model')
plt.plot(G_meas.real, G_meas.imag, 'bo', label='Measured')
plt.xlabel("Real(G)")
plt.ylabel("Imag(G)")
plt.title("Nyquist Plot — Measured vs. Modeled FRF")
plt.axis('equal')
plt.grid(True)
plt.legend()

# --- Plot magnitude comparison ---
plt.figure(figsize=(7,5))
plt.semilogx(freqs, np.abs(G_meas), 'bo-', label='Measured')
plt.semilogx(freqs, np.abs(G_model), 'r-', label='Model')
plt.xlabel("Frequency (Hz)")
plt.ylabel("|G(jω)| (rad/N·m)")
plt.title("Magnitude Comparison")
plt.grid(True, which='both')
plt.legend()

# --- Plot phase comparison ---
plt.figure(figsize=(7,5))
plt.semilogx(freqs, np.degrees(np.angle(G_meas)), 'bo-', label='Measured')
plt.semilogx(freqs, np.degrees(np.angle(G_model)), 'r-', label='Model')
plt.xlabel("Frequency (Hz)")
plt.ylabel("Phase (deg)")
plt.title("Phase Comparison")
plt.grid(True, which='both')
plt.legend()

plt.tight_layout()
plt.show()
