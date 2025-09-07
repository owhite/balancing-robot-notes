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
            pos = data.get("pos", None)
            vel = data.get("vel", None)
            print(pos, vel)
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
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
fig.subplots_adjust(hspace=0.4)

ln1, = ax1.plot([], [], lw=1, label="Position (rad)")
ax1.set_ylim(0, 2 * np.pi)
ax1.set_title("Position (0 – 2π)")
ax1.set_xlabel("Sample")
ax1.set_ylabel("rad")
ax1.legend(loc="upper right")

ln2, = ax2.plot([], [], lw=1, color="orange", label="Velocity (rad/s)")
ax2.set_title("Velocity")
ax2.set_xlabel("Sample")
ax2.set_ylabel("rad/s")
ax2.legend(loc="upper right")

# === Plot update loop ===
while True:
    with lock:
        if time_buf:
            ln1.set_data(time_buf, pos_buf)
            ln2.set_data(time_buf, vel_buf)
            ax1.set_xlim(max(0, time_buf[-1] - WINDOW), time_buf[-1])
            ax2.set_xlim(max(0, time_buf[-1] - WINDOW), time_buf[-1])

    plt.pause(0.05)
