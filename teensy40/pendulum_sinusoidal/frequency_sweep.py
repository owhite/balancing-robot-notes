#!/usr/bin/env python3
"""
frequency_sweep.py — Enhanced Version

Automated frequency response experiment for the torque-driven pendulum.
Performs multiple sweeps over a frequency range, averages amplitude and phase,
and plots the Bode-style magnitude and phase response.

Usage:
    python frequency_sweep.py <serial_port>
"""

import sys, json, serial, time, math
import matplotlib.pyplot as plt
import numpy as np

# --- ESC Configuration ---
PARAM_FILE = "sweep_params.json"

# --- Load parameters from file ---
def load_params():
    try:
        with open(PARAM_FILE, "r") as f:
            return json.load(f)
    except:
        return {
            "Kt": 0.056,            # Nm/A, torque constant
            "I_MAX": 30.0,          # Max amps used by ESC controller
            "torque": 0.1,          # Torque amplitude [N·m]
            "freq_start": 0.3,      # Start frequency [Hz]
            "freq_end": 8.0,        # End frequency [Hz]
            "freq_step": 0.5,       # Step between test frequencies [Hz]
            "duration_ms": 5000,    # Duration of each run
            "avg_runs": 2,          # Number of repeats per frequency
            "port": "/dev/cu.usbmodem178888901"
        }

params = load_params()

# --- Open serial port ---
port = sys.argv[1] if len(sys.argv) > 1 else params["port"]
ser = serial.Serial(port, 115200, timeout=0.1)

# --- Generate frequency list ---
freqs = np.arange(
    params["freq_start"],
    params["freq_end"] + params["freq_step"] / 2,
    params["freq_step"]
)

# --- Storage for results ---
results = []

# --- Sweep loop ---
for freq in freqs:
    amp_vals = []
    phase_vals = []

    print(f"=== Frequency {freq:.2f} Hz ===")
    for trial in range(params["avg_runs"]):
        print(f" Trial {trial+1}/{params['avg_runs']}")

        # Reset samples to prevent carryover between trials
        samples = []

        # Compute normalized ESC command amplitude
        cmd_amp = params["torque"] / (params["I_MAX"] * params["Kt"])

        msg = {
            "cmd": "send",
            "amp_command": cmd_amp,              # normalized ESC command [-1..1]
            "freq_hz": float(freq),
            "duration_us": params["duration_ms"] * 1000
        }

        print(f"→ TX: {msg}")
        ser.write((json.dumps(msg) + "\n").encode("utf-8"))

        # --- Wait for data to arrive ---
        buf = ""
        start_time = time.time()
        while True:
            if time.time() - start_time > (params["duration_ms"] / 1000.0 + 3):
                print(" Timeout waiting for response")
                break

            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            if line.startswith("{") and "samples" in line:
                buf = line
                while True:
                    chunk = ser.readline().decode("utf-8", errors="ignore")
                    if not chunk:
                        break
                    buf += chunk
                    try:
                        data = json.loads(buf)
                        break
                    except json.JSONDecodeError:
                        continue
                if "samples" in data:
                    samples = data["samples"]
                    break

        # --- Process response data ---
        if not samples:
            continue

        t_vals = np.array([s["t"] * 1e-6 for s in samples])  # µs → s
        pos_vals = np.array([s["pos"] for s in samples])
        torque_vals = np.array([s["torque"] * params["Kt"] * params["I_MAX"] for s in samples])  # [Nm]

        # Compute FFT-based amplitude and phase
        n = len(pos_vals)
        dt = np.mean(np.diff(t_vals))
        fs = 1.0 / dt
        freqs_fft = np.fft.rfftfreq(n, d=dt)
        pos_fft = np.fft.rfft(pos_vals)
        torque_fft = np.fft.rfft(torque_vals)

        idx = np.argmin(np.abs(freqs_fft - freq))
        amp_pos = 2 * abs(pos_fft[idx]) / n
        amp_torque = 2 * abs(torque_fft[idx]) / n
        amp_ratio = amp_pos / amp_torque

        # --- Phase calculation with consistent reference ---
        phase_raw = np.angle(pos_fft[idx]) - np.angle(torque_fft[idx])
        phase_deg = math.degrees(phase_raw)

        # Wrap into [-180, 180] so direction conventions don't flip
        phase_deg = (phase_deg + 180) % 360 - 180

        print(f"  → amp={amp_pos:.3f} rad, phase={phase_deg:.1f}°")

        amp_vals.append(amp_ratio)
        phase_vals.append(phase_deg)

        # brief delay between trials
        time.sleep(1.0)

    # --- Average results over trials ---
    amp_mean = float(np.mean(amp_vals))
    phase_mean = float(np.mean(phase_vals))

    results.append({
        "freq_hz": float(freq),
        "amp_rad": amp_mean,      # output amplitude [rad]
        "phase_deg": phase_mean   # phase lag [deg]
    })

    print(f" ✓ Averaged: amp={amp_mean:.3f} rad | phase={phase_mean:.1f}°")

# --- Save results to file ---
with open("results_sweep.json", "w") as f:
    json.dump(results, f, indent=2)

# --- Plot Bode magnitude and phase ---
freqs_plot = [r["freq_hz"] for r in results]
amp_plot = [r["amp_rad"] for r in results]

# Unwrap phase to remove 360° jumps (e.g., from –179° → +180°) so the phase varies smoothly and reflects true continuous lag
#  Unwrap across all frequencies 

phase_raws = [r["phase_deg"] for r in results]
phase_unwrapped = np.unwrap(np.radians(phase_raws))
phase_plot = np.degrees(phase_unwrapped)

plt.figure(figsize=(7,7))
plt.subplot(2,1,1)
plt.semilogx(freqs_plot, amp_plot, 'o-')
plt.ylabel("Amplitude (rad)")
plt.title("Measured Frequency Response")
plt.grid(True, which='both')

plt.subplot(2,1,2)
plt.semilogx(freqs_plot, phase_plot, 'o-')
plt.xlabel("Frequency (Hz)")
plt.ylabel("Phase (deg)")
plt.grid(True, which='both')

plt.tight_layout()
plt.savefig("bode_plot_avg.png", dpi=150)
plt.show()
