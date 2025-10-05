#!/usr/bin/env python3
"""
fit_model_from_sweep.py

Use the frequency sweep data (results_sweep.json) to fit a second-order
pendulum model of the form:

    J * θ_ddot + b * θ_dot + k * θ = τ

=> Transfer function G(s) = θ(s)/τ(s) = (1/J) / (s^2 + (b/J)s + (k/J))

Fits J, b, and k = m*g*l using nonlinear least squares on magnitude + phase.
"""

import json, numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt

# === Load measured data ===
with open("results_sweep.json") as f:
    data = json.load(f)
freqs = np.array([d["freq_hz"] for d in data])
amp_meas = np.array([d["amp_rad"] for d in data])
phase_meas = np.array([d["phase_deg"] for d in data])

# --- normalize amplitude by input torque amplitude ---
#  update this number to match your sweep's torque amplitude
torque_amp = 0.10   # [N·m]
amp_meas = amp_meas / torque_amp

# --- unwrap phase to avoid ±180° discontinuities ---
phase_meas = np.unwrap(np.radians(phase_meas))
phase_meas = np.degrees(phase_meas)

# === theoretical model ===
def model(freq, J, b, k):
    w = 2 * np.pi * freq
    num = 1.0 / J
    denom = (k / J - w**2) + 1j * (b * w / J)
    G = num / denom
    amp = np.abs(G)
    phase = np.angle(G, deg=True)
    return amp, phase

# === residual function ===
def residual(params):
    J, b, k = params
    amp_pred, phase_pred = model(freqs, J, b, k)

    # unwrap model phase too
    phase_pred = np.unwrap(np.radians(phase_pred))
    phase_pred = np.degrees(phase_pred)

    # logarithmic amplitude error + scaled phase error
    amp_err = np.log10(amp_pred) - np.log10(amp_meas)
    phase_err = (phase_pred - phase_meas) / 180.0
    return np.concatenate([amp_err, phase_err])

# === fit with improved initial guesses ===
initial_guess = [0.02, 0.2, 9.0]  # J, b, k
result = least_squares(residual, initial_guess, bounds=(0, np.inf))
J_fit, b_fit, k_fit = result.x

# --- derived parameters ---
fn = 1 / (2 * np.pi) * np.sqrt(k_fit / J_fit)
zeta = b_fit / (2 * np.sqrt(J_fit * k_fit))

# --- compute residual RMS ---
residuals = residual(result.x)
residual_rms = float(np.sqrt(np.mean(residuals**2)))

print("=== Fitted Parameters ===")
print(f"J  = {J_fit:.6f}  kg·m²")
print(f"b  = {b_fit:.6f}  N·m·s/rad")
print(f"k  = {k_fit:.6f}  N·m/rad (≈ m*g*l)")
print(f"Natural frequency (Hz) ≈ {fn:.3f}")
print(f"Damping ratio ζ ≈ {zeta:.3f}")
print(f"Residual RMS: {residual_rms:.6f}")

# --- Save results ---
results_out = {
    "J": J_fit,
    "b": b_fit,
    "k": k_fit,
    "fn_hz": fn,
    "zeta": zeta,
    "residual_rms": residual_rms
}
with open("rotational_i_viscous.json", "w") as f:
    json.dump(results_out, f, indent=2)

# --- Plot overlay ---
amp_pred, phase_pred = model(freqs, J_fit, b_fit, k_fit)
plt.figure(figsize=(7,7))
plt.subplot(2,1,1)
plt.semilogx(freqs, amp_meas, 'o', label="Measured")
plt.semilogx(freqs, amp_pred, '-', label="Fitted model")
plt.ylabel("Amplitude (rad/N·m)")
plt.legend(); plt.grid(True, which='both')
plt.title("Pendulum Frequency Response Fit")

plt.subplot(2,1,2)
plt.semilogx(freqs, phase_meas, 'o', label="Measured")
plt.semilogx(freqs, phase_pred, '-', label="Fitted model")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Phase (deg)")
plt.legend(); plt.grid(True, which='both')
plt.tight_layout()
plt.savefig("fit_model_from_sweep.png", dpi=150)
plt.show()



