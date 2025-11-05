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

# === Buffers for each sender ===
data = {
    11: {"time": deque(maxlen=WINDOW), "pos": deque(maxlen=WINDOW), "vel": deque(maxlen=WINDOW)},
    12: {"time": deque(maxlen=WINDOW), "pos": deque(maxlen=WINDOW), "vel": deque(maxlen=WINDOW)}
}

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
            data_in = json.loads(line)
            sid = data_in.get("sender", None)
            pos = data_in.get("pos", None)
            vel = data_in.get("vel", None)
            if sid not in data:
                continue  # ignore unknown sender IDs

            with lock:
                data[sid]["time"].append(sample_idx)
                data[sid]["pos"].append(pos)
                data[sid]["vel"].append(vel)
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

# Two sender lines
pos_line_11, = ax1.plot([0, 0], [0, 1], lw=2, color="blue", label="Sender 11")
pos_line_12, = ax1.plot([0, 0], [0, 1], lw=2, color="orange", label="Sender 12")

ax1.legend(loc="upper right")

# Configure angular ticks (radians)
ax1.set_xticks([0, np.pi/2, np.pi, 3*np.pi/2])
ax1.set_xticklabels(["0", "π/2", "π", "3π/2"])
ax1.set_yticklabels([])
ax1.yaxis.grid(False)

# --- Scrolling position plot (time series) ---
ax2 = fig.add_subplot(2, 1, 2)
ln11, = ax2.plot([], [], lw=1.5, color="blue", label="Sender 11")
ln12, = ax2.plot([], [], lw=1.5, color="orange", label="Sender 12")
ax2.set_title("Position (scrolling)")
ax2.set_xlabel("Sample")
ax2.set_ylabel("rad")
ax2.set_ylim(0, 2 * np.pi)
ax2.legend(loc="upper right")

# --- Shared labels (top-right of figure) ---
pos_text = fig.text(0.75, 0.95, "", ha="left", va="top", fontsize=14, weight="bold")
vel_text = fig.text(0.75, 0.90, "", ha="left", va="top", fontsize=14, weight="bold")

# === Plot update loop ===
while True:
    with lock:
        # --- Circular plot updates ---
        pos11 = data[11]["pos"][-1] if data[11]["pos"] else 0.0
        pos12 = data[12]["pos"][-1] if data[12]["pos"] else 0.0

        pos_line_11.set_data([0, pos11], [0, 1])
        pos_line_12.set_data([0, pos12], [0, 1])

        pos_text.set_text(f"11: {pos11:.2f} rad | 12: {pos12:.2f} rad")

        # --- Scrolling position updates ---
        if data[11]["time"]:
            ln11.set_data(data[11]["time"], data[11]["pos"])
        if data[12]["time"]:
            ln12.set_data(data[12]["time"], data[12]["pos"])

        if data[11]["time"] or data[12]["time"]:
            xmax = sample_idx
            ax2.set_xlim(max(0, xmax - WINDOW), xmax)

        # --- Velocity label ---
        vel11 = data[11]["vel"][-1] if data[11]["vel"] else 0.0
        vel12 = data[12]["vel"][-1] if data[12]["vel"] else 0.0
        vel_text.set_text(f"11: {vel11:.2f} | 12: {vel12:.2f} rad/s")

    plt.pause(0.05)
