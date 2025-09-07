#!/usr/bin/env python3
import sys
import serial
import json
import matplotlib.pyplot as plt
from collections import deque
import numpy as np
import threading

# === Config ===
if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} SERIAL_PORT [BAUDRATE]")
    sys.exit(1)

PORT = sys.argv[1]
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
WINDOW = 1000  # rolling window length

# === Buffers (shared between threads) ===
time_buf = deque(maxlen=WINDOW)
pos_buf  = deque(maxlen=WINDOW)
vel_buf  = deque(maxlen=WINDOW)

sample_idx = 0
lock = threading.Lock()

# === Serial reader thread ===
def serial_reader():
    global sample_idx
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Opened {PORT} at {BAUD} baud. Listening for POSVEL JSON...")

    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue
        try:
            data = json.loads(line)
            pos = data.get("pos", None)  # radians (0–2π)
            vel = data.get("vel", None)  # rad/s

            with lock:
                time_buf.append(sample_idx)
                pos_buf.append(pos)
                vel_buf.append(vel)
                sample_idx += 1
        except json.JSONDecodeError:
            continue

# === Start background thread ===
t = threading.Thread(target=serial_reader, daemon=True)
t.start()

# === Plot setup ===
plt.ion()
fig = plt.figure(figsize=(10, 8))

# --- Circular (polar) position plot ---
ax1 = fig.add_subplot(2, 1, 1, polar=True)
ax1.set_title("Position (circular)", va="bottom")

# Needle line (from center outwards)
pos_line, = ax1.plot([0, 0], [0, 1], lw=2)

# Configure angular ticks (radians)
ax1.set_xticks([0, np.pi/2, np.pi, 3*np.pi/2])
ax1.set_xticklabels(["0", "π/2", "π", "3π/2"])

# Hide radial (concentric) labels and grid
ax1.set_yticklabels([])
ax1.yaxis.grid(False)

# --- Scrolling position plot (time series) ---
ax2 = fig.add_subplot(2, 1, 2)
ln2, = ax2.plot([], [], lw=1, color="blue", label="Position (rad)")
ax2.set_title("Position (scrolling)")
ax2.set_xlabel("Sample")
ax2.set_ylabel("rad")
ax2.set_ylim(0, 2 * np.pi)  # fixed range 0–2π
ax2.legend(loc="upper right")

# --- Shared labels (top-right of entire figure) ---
pos_text = fig.text(0.75, 0.95, "", ha="left", va="top",
                    fontsize=16, weight="bold")
vel_text = fig.text(0.75, 0.90, "", ha="left", va="top",
                    fontsize=16, weight="bold")

# === Plot update loop ===
while True:
    with lock:
        if time_buf:
            # --- Position update (circular plot) ---
            pos = pos_buf[-1] if pos_buf else 0.0
            pos_line.set_data([0, pos], [0, 1])  # angle=pos, radius=1
            pos_text.set_text(f"Position: {pos:.2f} rad")

            # --- Position update (scrolling plot) ---
            ln2.set_data(time_buf, pos_buf)
            ax2.set_xlim(max(0, time_buf[-1] - WINDOW), time_buf[-1])

            # --- Velocity label ---
            vel = vel_buf[-1] if vel_buf else 0.0
            vel_text.set_text(f"Velocity: {vel:.1f} rad/s")

    plt.pause(0.05)
