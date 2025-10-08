#!/usr/bin/env python3
"""
torque_sinewave_with_model.py

Runs a sinewave torque experiment on the Teensy-controlled pendulum and overlays
real measured data with simulated model data computed from rotational_i_viscous.json.

Usage:
    python torque_sinewave_with_model.py <serial_port> rotational_i_viscous.json
"""

import sys, json, serial, math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox, Button
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d

PARAM_FILE = "params.json"

# --- Load experimental parameters ---
def load_params():
    try:
        with open(PARAM_FILE, "r") as f:
            return json.load(f)
    except:
        return {
            "Kt": 0.056,          # Nm/A
            "torque": 0.1,        # Nm
            "I_MAX": 30.0,
            "freq_hz": 1.0,
            "duration_ms": 5000,
            "port": "/dev/cu.usbmodem178888901"
        }

params = load_params()

model_file = sys.argv[1]

with open(model_file, "r") as f:
    model = json.load(f)
J = model["J"]
b = model["b"]
k = model["k"]

# --- Open serial connection ---
port = params["port"]
ser = serial.Serial(port, 115200, timeout=0.1)

burst_data = None

# --- Setup figure with 3 subplots ---
fig, (ax_torque, ax_pos, ax_vel) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
plt.subplots_adjust(left=0.1, right=0.75, top=0.92, bottom=0.1)

# Torque
line_torque, = ax_torque.plot([], [], color="tab:red", label="Measured Torque")
line_torque_sim, = ax_torque.plot([], [], color="tab:pink", linestyle="--", label="Simulated Torque")
ax_torque.set_title("Torque Input vs Time")
ax_torque.set_ylabel("Torque (Nm)")
ax_torque.grid(True)
ax_torque.legend()

# Position
line_pos, = ax_pos.plot([], [], color="tab:green", label="Measured Position")
line_pos_sim, = ax_pos.plot([], [], color="tab:olive", linestyle="--", label="Simulated Position")
ax_pos.set_title("Pendulum Position vs Time")
ax_pos.set_ylabel("Position (rad)")
ax_pos.grid(True)
ax_pos.legend()

# Velocity
line_vel, = ax_vel.plot([], [], color="tab:blue", label="Measured Velocity")
line_vel_sim, = ax_vel.plot([], [], color="tab:cyan", linestyle="--", label="Simulated Velocity")
ax_vel.set_title("Pendulum Velocity vs Time")
ax_vel.set_xlabel("t (ms)")
ax_vel.set_ylabel("Velocity (rad/s)")
ax_vel.grid(True)
ax_vel.legend()

# --- UI Controls ---
axbox_Kt       = plt.axes([0.8, 0.90, 0.15, 0.05])
axbox_amp      = plt.axes([0.8, 0.82, 0.15, 0.05])
axbox_freq     = plt.axes([0.8, 0.74, 0.15, 0.05])
axbox_duration = plt.axes([0.8, 0.66, 0.15, 0.05])
axbutton       = plt.axes([0.8, 0.54, 0.15, 0.07])

tb_Kt = TextBox(axbox_Kt, "", initial=str(params["Kt"]))
axbox_Kt.set_title("Kt (Nm/A)", fontsize=10)

tb_torque = TextBox(axbox_amp, "", initial=str(params["torque"]))
axbox_amp.set_title("torque (Nm)", fontsize=10)

tb_freq = TextBox(axbox_freq, "", initial=str(params["freq_hz"]))
axbox_freq.set_title("freq_hz", fontsize=10)

tb_duration = TextBox(axbox_duration, "", initial=str(params["duration_ms"]))
axbox_duration.set_title("duration_ms", fontsize=10)

button = Button(axbutton, "Run")

# --- Button handler ---
def save_and_run(event):
    params["Kt"] = float(tb_Kt.text)
    params["torque"] = float(tb_torque.text)
    params["freq_hz"] = float(tb_freq.text)
    params["duration_ms"] = int(tb_duration.text)

    with open(PARAM_FILE, "w") as f:
        json.dump(params, f, indent=2)

    cmd_amp = params["torque"] / (params["I_MAX"] * params["Kt"])

    msg = {
        "cmd": "send",
        "amp_command": cmd_amp,               # normalized ESC command [-1..1]
        "freq_hz": params["freq_hz"],
        "duration_us": params["duration_ms"] * 1000
    }

    print(f"TX: {msg}")
    ser.write((json.dumps(msg) + "\n").encode("utf-8"))

button.on_clicked(save_and_run)

# --- Update loop ---
def update(frame):
    global burst_data

    while ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
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

        if "cmd" in data and data["cmd"] == "PRINT" and "note" in data:
            print("RX:", data["note"])

    if burst_data:
        # --- Convert to arrays ---
        t0 = burst_data[0]["t"]
        t_vals = np.array([(s["t"] - t0) * 1e-3 for s in burst_data])  # µs → ms
        torque_vals = np.array([s["torque"] * params["Kt"] * params["I_MAX"] for s in burst_data])
        pos_vals = np.array([s["pos"] for s in burst_data])
        vel_vals = np.array([s["vel"] for s in burst_data])

        # --- Simulate response using fitted model ---
        tau_func = interp1d(t_vals * 1e-3, torque_vals, fill_value="extrapolate")  # seconds
        def dynamics(t, y):
            theta, theta_dot = y
            theta_ddot = (tau_func(t) - b * theta_dot - k * theta) / J
            return [theta_dot, theta_ddot]

        y0 = [pos_vals[0], vel_vals[0]]
        sol = solve_ivp(dynamics, [t_vals[0]*1e-3, t_vals[-1]*1e-3], y0, t_eval=t_vals*1e-3)
        theta_sim, theta_dot_sim = sol.y

        # --- Update plots (measured + simulated) ---
        # line_torque.set_data(t_vals, torque_vals)
        line_torque_sim.set_data(t_vals, torque_vals)  # identical torque input
        line_pos.set_data(t_vals, pos_vals)
        line_pos_sim.set_data(t_vals, theta_sim)
        line_vel.set_data(t_vals, vel_vals)
        line_vel_sim.set_data(t_vals, theta_dot_sim)

        # --- Autoscale axes based on simulation range (and measured data for context) ---
        ax_torque.set_xlim(np.min(t_vals), np.max(t_vals))
        ax_pos.set_xlim(np.min(t_vals), np.max(t_vals))
        ax_vel.set_xlim(np.min(t_vals), np.max(t_vals))

        for ax, measured, simulated in [
                (ax_torque, torque_vals, torque_vals),
                (ax_pos, pos_vals, theta_sim),
                (ax_vel, vel_vals, theta_dot_sim)
        ]:
            combined = np.concatenate((measured, simulated))
            ymin, ymax = np.min(combined), np.max(combined)
            margin = 0.1 * (ymax - ymin) if ymax > ymin else 1.0
            ax.set_ylim(ymin - margin, ymax + margin)

        burst_data = None

    return (line_torque, line_pos, line_vel, line_torque_sim, line_pos_sim, line_vel_sim)

ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
plt.show()
