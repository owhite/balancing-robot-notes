#!/usr/bin/env python3
import sys, json, serial, math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox, Button

# --- Motor Torque Command Calculation ---
# 1. From motor specs we know Kv (RPM/V). The torque constant Kt is its reciprocal:
#        Kt = 60 / (2π * Kv)   [Nm/A]
#    For Kv = 170 RPM/V, Kt ≈ 0.056 Nm/A.
#
# 2. To request a desired torque T (in Nm), compute the required phase current:
#        I = T / Kt
#    Example: For T = 0.2 Nm, I ≈ 3.57 A.
#
# 3. The ESC command is normalized from -1.0 to +1.0, where ±1.0 corresponds
#    to ±I_max (the max current limit set in the ESC configuration).
#        command = I / I_max
#    Example: With I_max = 30 A, command = 3.57 / 30 ≈ 0.12.
#
# => So sending a normalized torque command of ~0.12 will request ~0.2 Nm.

# --- ESC Configuration ---
I_MAX = 30.0  # Amps, max current limit configured in ESC

if len(sys.argv) < 2:
    print("Usage: python torque_response.py <serial_port>")
    sys.exit(1)

port = sys.argv[1]
ser = serial.Serial(port, 115200, timeout=0.1)

PARAM_FILE = "params.json"

# --- Load parameters from file ---
def load_params():
    try:
        with open(PARAM_FILE, "r") as f:
            return json.load(f)
    except:
        return {"Kt": 0.056, "Nm": 0.2, "pulse_ms": 85000, "total_ms": 170000}

params = load_params()

# --- Burst data container ---
burst_data = None

# --- Setup figure with 3 subplots ---
fig, (ax_torque, ax_pos, ax_vel) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
plt.subplots_adjust(left=0.1, right=0.75, top=0.92, bottom=0.1)

# Torque input
line_torque, = ax_torque.plot([], [], color="tab:red", label="Torque Command")
ax_torque.set_title("Torque Input vs Time")
ax_torque.set_ylabel("Torque (Nm)")
ax_torque.grid(True)
ax_torque.legend()

# Position
line_pos, = ax_pos.plot([], [], color="tab:green", label="Position")
ax_pos.set_title("Pendulum Position vs Time")
ax_pos.set_ylabel("Position (rad)")
ax_pos.grid(True)
ax_pos.legend()

# Velocity
line_vel, = ax_vel.plot([], [], color="tab:blue", label="Velocity")
ax_vel.set_title("Pendulum Velocity vs Time")
ax_vel.set_xlabel("t (ms)")
ax_vel.set_ylabel("Velocity (rad/s)")
ax_vel.grid(True)
ax_vel.legend()

# --- UI Controls ---
axbox_Kt      = plt.axes([0.8, 0.82, 0.15, 0.05])
axbox_Nm      = plt.axes([0.8, 0.74, 0.15, 0.05])
axbox_pulse   = plt.axes([0.8, 0.66, 0.15, 0.05])
axbox_total   = plt.axes([0.8, 0.58, 0.15, 0.05])
axbutton      = plt.axes([0.8, 0.46, 0.15, 0.07])

# --- TextBoxes with labels above ---
tb_Kt    = TextBox(axbox_Kt, "", initial=str(params["Kt"]))
axbox_Kt.set_title("Kt", fontsize=10)

tb_Nm    = TextBox(axbox_Nm, "", initial=str(params["Nm"]))
axbox_Nm.set_title("Nm", fontsize=10)

tb_pulse = TextBox(axbox_pulse, "", initial=str(params["pulse_ms"]))
axbox_pulse.set_title("pulse_ms", fontsize=10)

tb_total = TextBox(axbox_total, "", initial=str(params["total_ms"]))
axbox_total.set_title("total_ms", fontsize=10)

button   = Button(axbutton, "Run")

def save_and_run(event):
    global start_pos, pi2_time_ms, pi2_cross_pos, hline, vline

    # --- Reset state so multiple runs work correctly ---
    start_pos = None
    pi2_time_ms = None
    pi2_cross_pos = None
    label.set_text("π/2 Time: --- ms")


    # remove old crosshair lines
    if hline is not None:
        hline.remove()
        hline = None
    if vline is not None:
        vline.remove()
        vline = None

    params["Kt"] = float(tb_Kt.text)
    params["Nm"] = float(tb_Nm.text)
    params["pulse_ms"] = int(tb_pulse.text)
    params["total_ms"] = int(tb_total.text)
    with open(PARAM_FILE, "w") as f:
        json.dump(params, f, indent=2)

    # User-specified torque request (Nm)
    desired_torque_Nm = params["Nm"]

    # Convert torque → current → normalized command
    I = desired_torque_Nm / params["Kt"]
    cmd = (I / I_MAX)

    msg = {
        "cmd": "send",
        "pulse_torque": cmd,
        "pulse_us": params["pulse_ms"] * 1000,
        "total_us": params["total_ms"] * 1000
    }
    print("TX: ", msg)
    ser.write((json.dumps(msg) + "\n").encode("utf-8"))

button.on_clicked(save_and_run)

# --- PI/2 travel time tracking (accumulated distance) ---
start_pos = None
pi2_time_ms = None
pi2_cross_pos = None
ax_label = plt.axes([0.8, 0.36, 0.15, 0.05])  # area for text display
label = ax_label.text(0.0, 0.5, "π/2 Time: --- ms", transform=ax_label.transAxes, fontsize=14)
ax_label.axis("off")

# Line handles
hline = None
vline = None

# --- Utility: unwrap angle difference safely ---
def angle_diff(a, b):
    """Return minimal signed difference between two angles (wrap at 2π)."""
    return (a - b + math.pi) % (2*math.pi) - math.pi

# --- Update loop ---
def update(frame):
    global burst_data, start_pos, pi2_time_ms, pi2_cross_pos, hline, vline

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
        if "cmd" in data and data["cmd"] == "PRINT":
            if "note" in data :
                print("RX:", data["note"], " :: ", data)

    if burst_data:
        t_vals = [s["t"] * 1e-3 for s in burst_data]  # convert µs → ms
        torque_vals = [s["torque"] * params["Kt"] * I_MAX for s in burst_data]  # convert back to Nm
        pos_vals = [s["pos"] for s in burst_data]
        vel_vals = [s["vel"] for s in burst_data]

        # --- Track accumulated travel distance ---
        if start_pos is None:
            start_pos = pos_vals[0]

        if pi2_time_ms is None:
            total_dist = 0.0
            last_angle = start_pos
            for i in range(1, len(pos_vals)):
                diff = angle_diff(pos_vals[i], last_angle)
                total_dist += abs(diff)
                last_angle = pos_vals[i]

                if total_dist >= math.pi/2:
                    pi2_time_ms = t_vals[i]
                    pi2_cross_pos = pos_vals[i]
                    break

        if pi2_time_ms is not None:
            label.set_text(f"π/2 Time: {pi2_time_ms:.1f} ms")
            # Draw crosshair lines once
            if hline is None and pi2_cross_pos is not None:
                hline = ax_pos.axhline(y=pi2_cross_pos, color="red", linestyle="--", label="π/2 crossing")
            if vline is None:
                vline = ax_pos.axvline(x=pi2_time_ms, color="red", linestyle=":", label="π/2 time")
            ax_pos.legend()

        line_torque.set_data(t_vals, torque_vals)
        line_pos.set_data(t_vals, pos_vals)
        line_vel.set_data(t_vals, vel_vals)

        # --- Rescale axes based on min/max of data ---
        ax_torque.set_xlim(min(t_vals), max(t_vals))
        ax_pos.set_xlim(min(t_vals), max(t_vals))
        ax_vel.set_xlim(min(t_vals), max(t_vals))

        # Torque
        ymin, ymax = min(torque_vals), max(torque_vals)
        margin = 0.1 * (ymax - ymin) if ymax > ymin else 1.0
        ax_torque.set_ylim(ymin - margin, ymax + margin)

        # Position
        ymin, ymax = min(pos_vals), max(pos_vals)
        margin = 0.1 * (ymax - ymin) if ymax > ymin else 1.0
        ax_pos.set_ylim(ymin - margin, ymax + margin)

        # Velocity
        ymin, ymax = min(vel_vals), max(vel_vals)
        margin = 0.1 * (ymax - ymin) if ymax > ymin else 1.0
        ax_vel.set_ylim(ymin - margin, ymax + margin)

        burst_data = None

    return (line_torque, line_pos, line_vel, label)

ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
plt.show()
