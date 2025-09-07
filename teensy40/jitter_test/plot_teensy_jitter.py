#!/usr/bin/env python3
import sys
import serial
import json
import matplotlib.pyplot as plt
from collections import deque
import numpy as np

# === Config ===
if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} SERIAL_PORT [BAUDRATE]")
    sys.exit(1)

PORT = sys.argv[1]
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
WINDOW = 1000  # rolling window length

# === Set up serial ===
ser = serial.Serial(PORT, BAUD, timeout=1)

# === Rolling buffers ===
time_buf = deque(maxlen=WINDOW)
pos_buf  = deque(maxlen=WINDOW)
vel_buf  = deque(maxlen=WINDOW)

plt.ion()
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
fig.subplots_adjust(hspace=0.4)

# --- Position plot ---
ln1, = ax1.plot([], [], lw=1, label="Position (rad)")
ax1.set_ylim(0, 2 * np.pi)
ax1.set_title("Position (0 – 2π)", fontsize=12, pad=12)
ax1.set_xlabel("Sample")
ax1.set_ylabel("rad")
ax1.legend(loc="upper right")

# --- Velocity plot ---
ln2, = ax2.plot([], [], lw=1, color="orange", label="Velocity (rad/s)")
ax2.set_title("Velocity", fontsize=12, pad=12)
ax2.set_xlabel("Sample")
ax2.set_ylabel("rad/s")
ax2.legend(loc="upper right")

# === Main loop ===
sample_idx = 0
while True:
    line = ser.readline().decode(errors="ignore").strip()
    if not line:
        continue

    try:
        data = json.loads(line)
        pos = data.get("pos", 0.0)
        vel = data.get("vel", 0.0)

        time_buf.append(sample_idx)
        pos_buf.append(pos)
        vel_buf.append(vel)
        sample_idx += 1

        # update plots
        ln1.set_data(time_buf, pos_buf)
        ln2.set_data(time_buf, vel_buf)

        ax1.set_xlim(max(0, sample_idx - WINDOW), sample_idx)
        ax2.set_xlim(max(0, sample_idx - WINDOW), sample_idx)

        plt.pause(0.01)

    except json.JSONDecodeError:
        continue
