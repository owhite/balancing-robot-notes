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
WINDOW = 2000  # rolling window length for time series

# === Set up serial ===
ser = serial.Serial(PORT, BAUD, timeout=1)

# === Rolling buffers ===
period_buf = deque(maxlen=WINDOW)
err_buf    = deque(maxlen=WINDOW)

plt.ion()
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
# Leave room on the right for sidebar, and add vertical spacing for titles
fig.subplots_adjust(right=0.75, hspace=0.6, top=0.92, bottom=0.08)

# --- Time series plots ---
ln1, = ax1.plot([], [], lw=1)
ax1.set_title("Period (µs)", fontsize=12, pad=12)
ax1.set_xlabel("Sample")
ax1.set_ylabel("Period [µs]")

ln2, = ax2.plot([], [], lw=1, color="red")
ax2.set_title("Jitter (Error vs Target)", fontsize=12, pad=12)
ax2.set_xlabel("Sample")
ax2.set_ylabel("Error [µs]")

# --- Violin plot ---
ax3.set_title("Jitter Distribution (Violin Plot)", fontsize=12, pad=12)
ax3.set_ylabel("Error [µs]")

# --- Sidebar stats text ---
stats_text = fig.text(0.8, 0.5, "", va="center", ha="left", fontsize=10,
                      family="monospace")

def update_stats():
    if not err_buf:
        return ""

    arr = np.array(err_buf)
    mean = np.mean(arr)
    std = np.std(arr)
    p99 = np.percentile(arr, 99)
    mn, mx = np.min(arr), np.max(arr)

    return (f"Samples: {len(arr):>6}\n"
            f"Mean:    {mean:8.2f} µs\n"
            f"StdDev:  {std:8.2f} µs\n"
            f"99%ile:  {p99:8.2f} µs\n"
            f"Min:     {mn:8.2f} µs\n"
            f"Max:     {mx:8.2f} µs")

def update_plot():
    # update time series
    ln1.set_data(range(len(period_buf)), period_buf)
    ln2.set_data(range(len(err_buf)), err_buf)

    ax1.relim(); ax1.autoscale_view()
    ax2.relim(); ax2.autoscale_view()

    # update violin plot
    ax3.cla()
    if len(err_buf) > 10:  # need enough points
        ax3.violinplot(err_buf, showmeans=True, showextrema=True, showmedians=True)
        ax3.set_title("Jitter Distribution (Violin Plot)", fontsize=12, pad=12)
        ax3.set_ylabel("Error [µs]")

    # update stats sidebar
    stats_text.set_text(update_stats())

    plt.pause(0.01)

print(f"Listening on {PORT} at {BAUD} baud...")
try:
    while True:
        line = ser.readline().decode("utf-8").strip()
        if not line:
            continue
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            continue

        if "period_us" in data:
            period_buf.append(data["period_us"])
            err_buf.append(data["err_us"])
            update_plot()

        elif "warn" in data:
            print("Warning:", data)
except KeyboardInterrupt:
    print("Exiting...")
    ser.close()
