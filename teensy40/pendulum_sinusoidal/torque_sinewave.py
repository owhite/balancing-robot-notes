#!/usr/bin/env python3
#!/usr/bin/env python3
import sys, json, serial, math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox, Button

# --- ESC Configuration ---
I_MAX = 30.0  # Amps, max current limit configured in ESC

PARAM_FILE = "params.json"

# --- Load parameters from file ---
def load_params():
    try:
        with open(PARAM_FILE, "r") as f:
            return json.load(f)
    except:
        # Default parameters if file missing
        return {
            "Kt": 0.056,          # Nm/A, torque constant
            "amp_torque": 0.1,    # Nm
            "freq_hz": 1.0,
            "duration_ms": 5000, 
            "port": "/dev/cu.usbmodem178888901"
        }

params = load_params()
port = sys.argv[1] if len(sys.argv) > 1 else params["port"]
ser = serial.Serial(port, 115200, timeout=0.1)

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
axbox_Kt       = plt.axes([0.8, 0.90, 0.15, 0.05])
axbox_amp      = plt.axes([0.8, 0.82, 0.15, 0.05])
axbox_freq     = plt.axes([0.8, 0.74, 0.15, 0.05])
axbox_duration = plt.axes([0.8, 0.66, 0.15, 0.05])
axbutton       = plt.axes([0.8, 0.54, 0.15, 0.07])

# TextBoxes with labels above
tb_Kt = TextBox(axbox_Kt, "", initial=str(params["Kt"]))
axbox_Kt.set_title("Kt (Nm/A)", fontsize=10)

tb_amp = TextBox(axbox_amp, "", initial=str(params["amp_torque"]))
axbox_amp.set_title("amp_torque (Nm)", fontsize=10)

tb_freq = TextBox(axbox_freq, "", initial=str(params["freq_hz"]))
axbox_freq.set_title("freq_hz", fontsize=10)

tb_duration = TextBox(axbox_duration, "", initial=str(params["duration_ms"]))
axbox_duration.set_title("duration_ms", fontsize=10)

button = Button(axbutton, "Run")

# --- Button handler ---
def save_and_run(event):
    # Update params from textboxes
    params["Kt"] = float(tb_Kt.text)
    params["amp_torque"] = float(tb_amp.text)
    params["freq_hz"] = float(tb_freq.text)
    params["duration_ms"] = int(tb_duration.text)

    # Save params to file
    with open(PARAM_FILE, "w") as f:
        json.dump(params, f, indent=2)

    # Compute equivalent ESC command amplitude
    I = params["amp_torque"] / params["Kt"]
    cmd_amp = I / I_MAX

    msg = {
        "cmd": "send",
        "amp_torque": params["amp_torque"],   # physical torque amplitude [Nm]
        "amp_command": cmd_amp,               # normalized ESC command [-1..1]
        "freq_hz": params["freq_hz"],
        "duration_us": params["duration_ms"] * 1000
    }

    print(f"TX: {msg}")
    print(f"   => I = {I:.3f} A,  cmd = {cmd_amp:.3f}")
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

        if "cmd" in data and data["cmd"] == "PRINT":
            if "note" in data:
                print("RX:", data["note"])

    if burst_data:
        # Use Teensy timestamps for x-axis
        t0 = burst_data[0]["t"]
        t_vals = [(s["t"] - t0) * 1e-3 for s in burst_data]  # µs → ms

        torque_vals = [s["torque"] * params["Kt"] * I_MAX for s in burst_data]  # Nm
        pos_vals = [s["pos"] for s in burst_data]
        vel_vals = [s["vel"] for s in burst_data]

        # Plot data
        line_torque.set_data(t_vals, torque_vals)
        line_pos.set_data(t_vals, pos_vals)
        line_vel.set_data(t_vals, vel_vals)

        # Axis scaling
        ax_torque.set_xlim(min(t_vals), max(t_vals))
        ax_pos.set_xlim(min(t_vals), max(t_vals))
        ax_vel.set_xlim(min(t_vals), max(t_vals))

        for ax, vals in [(ax_torque, torque_vals), (ax_pos, pos_vals), (ax_vel, vel_vals)]:
            ymin, ymax = min(vals), max(vals)
            margin = 0.1 * (ymax - ymin) if ymax > ymin else 1.0
            ax.set_ylim(ymin - margin, ymax + margin)

        burst_data = None

    return (line_torque, line_pos, line_vel)

ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
plt.show()
