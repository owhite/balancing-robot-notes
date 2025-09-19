#!/usr/bin/env python3
import sys
import json
import serial
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

if len(sys.argv) < 2:
    print("Usage: python position_burst.py <serial_port>")
    sys.exit(1)

port = sys.argv[1]
ser = serial.Serial(port, 115200, timeout=0.1)

# --- Live scrolling buffer ---
history_len = 500  # number of points to keep in live plot
t_live = deque(maxlen=history_len)
pos_live = deque(maxlen=history_len)
set_live = deque(maxlen=history_len)  # fixed setpoint (π) in idle mode

# --- Burst data container ---
burst_data = None

# --- Setup figure with 3 subplots ---
fig, (ax_live, ax_burst, ax_veltorque) = plt.subplots(3, 1, figsize=(10, 12))

# Live plot (position + setpoint only)
line_pos, = ax_live.plot([], [], label="Position", color="tab:green")
line_set, = ax_live.plot([], [], label="Setpoint", color="tab:orange")
ax_live.set_title("Live Position (Idle Mode)")
ax_live.set_xlabel("t (us)")
ax_live.set_ylabel("Position (rad)")
ax_live.set_ylim(0.0, 2 * math.pi)
ax_live.legend()
ax_live.grid(True)

# Burst plot (position + setpoint + error)
line_burst_pos, = ax_burst.plot([], [], label="Position", color="tab:green")
line_burst_set, = ax_burst.plot([], [], label="Setpoint", color="tab:orange")
line_burst_err, = ax_burst.plot([], [], label="Error", color="tab:red")
ax_burst.set_title("Burst Data (Position/Setpoint/Error)")
ax_burst.set_xlabel("t (us)")
ax_burst.set_ylabel("Value (rad)")
ax_burst.set_ylim(-2 * math.pi, 2 * math.pi)
ax_burst.legend()
ax_burst.grid(True)

# Burst velocity/torque plot with twin y-axis
line_burst_vel, = ax_veltorque.plot([], [], label="Velocity", color="tab:blue")
ax_veltorque.set_title("Burst Data (Velocity & Torque)")
ax_veltorque.set_xlabel("t (us)")
ax_veltorque.set_ylabel("Velocity (rad/s)", color="tab:blue")
ax_veltorque.tick_params(axis="y", labelcolor="tab:blue")
ax_veltorque.grid(True)

# Create second axis for torque & PID terms
ax_torque = ax_veltorque.twinx()
line_burst_torque, = ax_torque.plot([], [], label="Torque", color="tab:red")
line_pterm, = ax_torque.plot([], [], label="P-term", color="tab:green", linestyle="--")
line_dterm, = ax_torque.plot([], [], label="D-term", color="tab:purple", linestyle="--")
ax_torque.set_ylabel("Torque / PID terms (Nm)", color="tab:red")
ax_torque.tick_params(axis="y", labelcolor="tab:red")

# Add legends
ax_veltorque.legend(loc="upper left")
ax_torque.legend(loc="upper right")

def update(frame):
    global burst_data

    # --- Read lines from serial ---
    while ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            # Maybe burst block (multi-line JSON)
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
                    burst_data = data["samples"]
            continue

        # --- Live streaming data (idle mode) ---
        if "pos" in data:
            t_live.append(data["t"])
            pos_live.append(data["pos"])
            set_live.append(math.pi)  # always π in idle mode

    # --- Update live plot ---
    if t_live:
        line_pos.set_data(t_live, pos_live)
        line_set.set_data(t_live, set_live)
        ax_live.set_xlim(max(t_live[0], t_live[-1] - 50000), t_live[-1])  # ~last 50ms

    # --- Update burst plots ---
    if burst_data:
        t_vals = [s["t"] for s in burst_data]
        pos_vals = [s["pos"] for s in burst_data]
        set_vals = [s["setpoint"] for s in burst_data]
        err_vals = [s["err"] for s in burst_data]
        vel_vals = [s["vel"] for s in burst_data]
        torque_vals = [s["torque"] for s in burst_data]

        # Optional: P and D terms, if logged
        p_vals = [s.get("p_term", 0.0) for s in burst_data]
        d_vals = [s.get("d_term", 0.0) for s in burst_data]

        # Position/setpoint/error
        line_burst_pos.set_data(t_vals, pos_vals)
        line_burst_set.set_data(t_vals, set_vals)
        line_burst_err.set_data(t_vals, err_vals)
        ax_burst.set_xlim(min(t_vals), max(t_vals))

        # Velocity/torque + terms
        line_burst_vel.set_data(t_vals, vel_vals)
        line_burst_torque.set_data(t_vals, torque_vals)
        line_pterm.set_data(t_vals, p_vals)
        line_dterm.set_data(t_vals, d_vals)

        ax_veltorque.set_xlim(min(t_vals), max(t_vals))

        # Scale velocity axis
        vmin, vmax = min(vel_vals), max(vel_vals)
        vrange = vmax - vmin if vmax != vmin else 1.0
        ax_veltorque.set_ylim(vmin - 0.1 * vrange, vmax + 0.1 * vrange)

        # Scale torque axis (includes torque + terms)
        all_torque = torque_vals + p_vals + d_vals
        tmin, tmax = min(all_torque), max(all_torque)
        trange = tmax - tmin if tmax != tmin else 1.0
        ax_torque.set_ylim(tmin - 0.1 * trange, tmax + 0.1 * trange)

        burst_data = None  # only draw once per burst

    return (line_pos, line_set,
            line_burst_pos, line_burst_set, line_burst_err,
            line_burst_vel, line_burst_torque, line_pterm, line_dterm)

ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()
