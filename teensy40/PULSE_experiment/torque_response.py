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

if len(sys.argv) < 2:
    print("Usage: python position_burst.py <serial_port>")
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
        return {"setpoint": math.pi, "p_term": 0.07, "d_term": 0.004}

params = load_params()

# --- Burst data container ---
burst_data = None
current_position = 0.0

# --- Setup figure with 2 subplots ---
fig, (ax_burst, ax_veltorque) = plt.subplots(2, 1, figsize=(10, 8))

# Shrink plot width to leave space for widgets on the right
plt.subplots_adjust(left=0.1, right=0.75, top=0.92, bottom=0.1)

# --- Position label (top of right column) ---
pos_text = fig.text(
    0.8, 0.92,
    f"Position: {current_position:.4f} rad | Setpoint: {params['setpoint']:.4f}",
    fontsize=12
)

# Burst plot
line_burst_pos, = ax_burst.plot([], [], label="Position", color="tab:green")
line_burst_set, = ax_burst.plot([], [], label="Setpoint", color="tab:orange")
line_burst_err, = ax_burst.plot([], [], label="Error", color="tab:red")
ax_burst.set_title("Burst Data (Position/Setpoint/Error)")
ax_burst.set_xlabel("t (seconds)")
ax_burst.set_ylabel("Value (rad)")
ax_burst.set_ylim(-2*math.pi, 2*math.pi)
ax_burst.legend()
ax_burst.grid(True)

# Burst velocity/torque
line_burst_vel, = ax_veltorque.plot([], [], label="Velocity", color="tab:blue")
ax_veltorque.set_title("Burst Data (Velocity & Torque)")
ax_veltorque.set_xlabel("t seconds")
ax_veltorque.set_ylabel("Velocity (rad/s)", color="tab:blue")
ax_veltorque.tick_params(axis="y", labelcolor="tab:blue")
ax_veltorque.grid(True)
ax_veltorque.set_ylim(-200, 200)

# Second axis for torque & PID terms
ax_torque = ax_veltorque.twinx()
line_burst_torque, = ax_torque.plot([], [], label="Torque", color="tab:red")
line_pterm, = ax_torque.plot([], [], label="P-term", color="tab:green", linestyle="--")
line_dterm, = ax_torque.plot([], [], label="D-term", color="tab:purple", linestyle="--")
ax_torque.set_ylabel("Torque / PID terms (Nm)", color="tab:red")
ax_torque.tick_params(axis="y", labelcolor="tab:red")
ax_torque.set_ylim(-.6, .6)
ax_veltorque.legend(loc="upper left")
ax_torque.legend(loc="upper right")

# --- UI Controls (TextBoxes + Button) ---
axbox_set = plt.axes([0.8, 0.82, 0.15, 0.05])
axbox_p   = plt.axes([0.8, 0.74, 0.15, 0.05])
axbox_i   = plt.axes([0.8, 0.66, 0.15, 0.05])
axbox_d   = plt.axes([0.8, 0.56, 0.15, 0.05])
axbutton  = plt.axes([0.8, 0.44, 0.15, 0.07])

tb_set = TextBox(axbox_set, "Setpnt", initial=f"{params['setpoint']:.4f}")
tb_p   = TextBox(axbox_p, "P-term", initial=str(params["p_term"]))
tb_i   = TextBox(axbox_i, "I-term", initial=str(params["i_term"]))
tb_d   = TextBox(axbox_d, "D-term", initial=str(params["d_term"]))
button = Button(axbutton, "Run") # this also saves to disk. 

def save_and_run(event):
    params["setpoint"] = float(tb_set.text)
    params["p_term"] = float(tb_p.text)
    params["i_term"] = float(tb_i.text)
    params["d_term"] = float(tb_d.text)
    with open(PARAM_FILE, "w") as f:
        json.dump(params, f, indent=2)
    # send JSON to Teensy
    msg = {
        "cmd": "send",
        "setpoint": params["setpoint"],
        "p_term": params["p_term"],
        "i_term": params["i_term"],
        "d_term": params["d_term"]
    }
    # print("SEND:", msg)
    ser.write((json.dumps(msg) + "\n").encode("utf-8"))

button.on_clicked(save_and_run)

# --- Update loop ---
def update(frame):
    global burst_data, current_position

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

        # live position update
        if "pos" in data:
            current_position = data["pos"]
            pos_text.set_text(f"Position: {current_position:.3f} rad")

        # debuggy stuff
        if "cmd" in data and data["cmd"] == "PRINT":
            pass
            # print("PRINT:", data)

    # burst plots
    if burst_data:
        t_vals = [s["t"] for s in burst_data]
        pos_vals = [s["pos"] for s in burst_data]
        set_vals = [s["setpoint"] for s in burst_data]
        err_vals = [s["err"] for s in burst_data]
        vel_vals = [s["vel"] for s in burst_data]
        torque_vals = [s["torque"] for s in burst_data]
        p_vals = [s.get("p_term", 0.0) for s in burst_data]
        d_vals = [s.get("d_term", 0.0) for s in burst_data]

        line_burst_pos.set_data(t_vals, pos_vals)
        line_burst_set.set_data(t_vals, set_vals)
        line_burst_err.set_data(t_vals, err_vals)
        ax_burst.set_xlim(min(t_vals), max(t_vals))

        line_burst_vel.set_data(t_vals, vel_vals)
        line_burst_torque.set_data(t_vals, torque_vals)
        line_pterm.set_data(t_vals, p_vals)
        line_dterm.set_data(t_vals, d_vals)
        ax_veltorque.set_xlim(min(t_vals), max(t_vals))

        burst_data = None

    return (line_burst_pos, line_burst_set, line_burst_err,
            line_burst_vel, line_burst_torque, line_pterm, line_dterm, pos_text)

ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
plt.show()
