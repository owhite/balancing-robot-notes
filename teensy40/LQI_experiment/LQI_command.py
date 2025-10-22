#!/usr/bin/env python3
import sys, json, serial, math
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox, Button
import numpy as np
from scipy.signal import cont2discrete
from scipy.linalg import solve_discrete_are

I_MAX = 30.0  # Amps, max current limit configured in ESC

def save_and_run(event, params, textboxes, label, axes_state, args, ser):
    """
    Handles 'Run' button click:
      - Reads updated parameters from UI textboxes
      - Saves JSON file
      - Sends command to serial port (currently commented)
      - Resets UI markers
    """
    start_pos, pi2_time_ms, pi2_cross_pos, hline, vline = axes_state

    # Reset tracking
    start_pos = None
    pi2_time_ms = None
    pi2_cross_pos = None

    # Remove old crosshairs
    if hline is not None:
        hline.remove()
        hline = None
    if vline is not None:
        vline.remove()
        vline = None

    # Unpack TextBoxes
    tb_qterm, tb_rterm, tb_torque, tb_theta, tb_total = textboxes

    # Update parameters from UI
    params["qterm"] = tb_qterm.text
    params["rterm"] = float(tb_rterm.text)
    params["torque"] = float(tb_torque.text)
    params["theta"] = float(tb_theta.text)
    params["total_ms"] = int(tb_total.text)

    with open(args.json_params, "w") as f:
        json.dump(params, f, indent=2)

    # COMPUTE cont2discrete() HERE
    # --- Extract numeric parameters ---
    Kt = float(params["Kt"])
    lam = float(params["lambda"])       # = b/J
    b = float(params["b_decay"])
    Ts = float(params["Ts"])

    # Recover inertia J = b / λ
    J = b / lam

    # Parse Q and R (allow string or list)
    try:
        Q_vals = json.loads(params["qterm"])
    except Exception:
        Q_vals = [100.0, 1.0, 50.0]

    Q = np.diag(Q_vals)
    R = np.array([[float(params["rterm"])]])

    # --- Continuous-time augmented model (LQI) ---
    A_c = np.array([[0, 1, 0],
                    [0, -lam, 0],
                    [ 1,  0, 0]])  
    B_c = np.array([[0],
                    [Kt / J],
                    [0]])

    # --- Discretize for Teensy loop ---
    Ad, Bd, _, _, _ = cont2discrete((A_c, B_c, np.eye(3), np.zeros((3,1))), Ts)

    # --- Solve discrete Riccati equation and compute gain ---
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.inv(Bd.T @ P @ Bd + R) @ (Bd.T @ P @ Ad)

    print(K.flatten())
    Kth_term, Kw_term, Ki_term = K.flatten()
    print(f"✅ Discrete LQI gains: Kθ={Kth_term:.4f}, Kω={Kw_term:.4f}, Ki={Ki_term:.4f}")

    # Optional: show in GUI label
    label.set_text(f"Kθ={Kth_term:.3f}\nKω={Kw_term:.3f}\nKi={Ki_term:.3f}")

    msg = {
        "cmd": "position", 
        "torque": float(params["torque"]),
        "total_us": int(params["total_ms"] * 1000),
        "user_Kth_term": round(float(Kth_term), 4), 
        "user_Kw_term": round(float(Kw_term),4), 
        "user_Ki_term": round(float(Ki_term),4), 
        "theta_ref": float(params["theta"])
    }

    print("TX:", msg)

    # CREATE JSON
    # {'cmd': 'position', 'torque': 0.2, 'total_us': 3000000000, 'user_Kth_term': 10.1479, 'user_Kw_term': 1.0603, 'user_Ki_term': 6.6391, 'theta_ref': 3.4}

    ser.write((json.dumps(msg) + "\n").encode("utf-8"))

    # Return updated axes state for reference
    return start_pos, pi2_time_ms, pi2_cross_pos, hline, vline


def main():
    parser = argparse.ArgumentParser(description="Pendulum live serial plotter and controller.")
    parser.add_argument("-p", "--port", required=True, help="Input serial port")
    parser.add_argument("-j", "--json_params", required=True, help="Input parameter data")
    args = parser.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.1)

    with open(args.json_params, "r") as f:
        params = json.load(f)

    # --- Setup plots ---
    fig, (ax_torque, ax_pos, ax_vel) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    plt.subplots_adjust(left=0.1, right=0.75, top=0.92, bottom=0.1)

    line_torque, = ax_torque.plot([], [], color="tab:red", label="Torque Command")
    ax_torque.set_title("Torque Input vs Time")
    ax_torque.set_ylabel("Torque (Nm)")
    ax_torque.grid(True)
    ax_torque.legend()

    line_pos, = ax_pos.plot([], [], color="tab:green", label="Position")
    ax_pos.set_title("Pendulum Position vs Time")
    ax_pos.set_ylabel("Position (rad)")
    ax_pos.grid(True)
    ax_pos.legend()

    line_vel, = ax_vel.plot([], [], color="tab:blue", label="Velocity")
    ax_vel.set_title("Pendulum Velocity vs Time")
    ax_vel.set_xlabel("t (ms)")
    ax_vel.set_ylabel("Velocity (rad/s)")
    ax_vel.grid(True)
    ax_vel.legend()

    # tb_qterm, tb_rterm, tb_torque, tb_theta, tb_total

    # --- UI controls ---
    axbox_qterm  = plt.axes([0.8, 0.90, 0.15, 0.05])
    axbox_rterm  = plt.axes([0.8, 0.82, 0.15, 0.05])
    axbox_torque = plt.axes([0.8, 0.74, 0.15, 0.05])
    axbox_theta  = plt.axes([0.8, 0.66, 0.15, 0.05])
    axbox_total  = plt.axes([0.8, 0.58, 0.15, 0.05])
    axbutton     = plt.axes([0.8, 0.4, 0.15, 0.07])

    tb_qterm  = TextBox(axbox_qterm,  "qterm",  initial=str(params["qterm"]))
    tb_rterm  = TextBox(axbox_rterm,  "rterm",  initial=str(params["rterm"]))
    tb_torque = TextBox(axbox_torque, "torque", initial=str(params["torque"]))
    tb_theta  = TextBox(axbox_theta,  "theta",  initial=str(params["theta"]))
    tb_total  = TextBox(axbox_total,  "total",  initial=str(params["total_ms"]))
    button    = Button(axbutton, "Run")

    # --- Label display ---
    ax_label = plt.axes([0.8, 0.46, 0.15, 0.05])
    label = ax_label.text(0.0, 0.5, "---", transform=ax_label.transAxes, fontsize=14)
    ax_label.axis("off")

    # --- Initialize state ---
    start_pos = None
    pi2_time_ms = None
    pi2_cross_pos = None
    hline = None
    vline = None
    axes_state = [start_pos, pi2_time_ms, pi2_cross_pos, hline, vline]
    textboxes = [tb_qterm, tb_rterm, tb_torque, tb_theta, tb_total]

    # --- Hook up callback ---
    button.on_clicked(lambda event: save_and_run(event, params, textboxes, label, axes_state, args, ser))

    # --- Animation loop ---
    burst_data = None

    def update(frame):
        nonlocal burst_data, start_pos, pi2_time_ms, pi2_cross_pos, hline, vline

        buf = ""
        while ser.in_waiting:
            chunk = ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
            buf += chunk

            if not buf.strip():
                continue
            try:
                data = json.loads(buf)
                if "samples" in data:
                    burst_data = data["samples"]
                    print(f"✅ Received burst of {len(burst_data)} samples.")

                    ### Compute inertial values here

                    # Convert data to numpy arrays for analysis
                    t = np.array([s["t"] * 1e-6 for s in burst_data])   # time in seconds
                    torque = np.array([s["torque"] for s in burst_data])
                    vel = np.array([s["vel"] for s in burst_data])

                    # --- Find when torque goes to zero (start of spin-down) ---
                    zero_idx = np.argmax(torque == 0)  # first index where torque == 0
                    if zero_idx <= 0 or zero_idx >= len(t) - 10:
                        # print("⚠️  Not enough spin-down data to estimate damping.")
                        pass
                    else:
                        msg = "report"
                        print(f"✅ {msg}")
                        label.set_text(msg)

                    buf = ""  # reset after full parse

            except json.JSONDecodeError:
                # wait for more data next frame
                continue

            # --- handle parsed data ---
            if "cmd" in data and data["cmd"] == "PRINT" and "note" in data:
                print("RX:", data["note"], "::", data)

        if burst_data:
            t_vals = [s["t"] * 1e-3 for s in burst_data]  # µs → ms
            torque_vals = [s["torque"] * I_MAX for s in burst_data]
            pos_vals = [s["pos"] for s in burst_data]
            vel_vals = [s["vel"] for s in burst_data]

            line_torque.set_data(t_vals, torque_vals)
            line_pos.set_data(t_vals, pos_vals)
            line_vel.set_data(t_vals, vel_vals)

            # Rescale axes dynamically
            for ax, vals in [(ax_torque, torque_vals), (ax_pos, pos_vals), (ax_vel, vel_vals)]:
                ax.set_xlim(min(t_vals), max(t_vals))
                ymin, ymax = min(vals), max(vals)
                margin = 0.1 * (ymax - ymin) if ymax > ymin else 1.0
                ax.set_ylim(ymin - margin, ymax + margin)

            burst_data = None

        # (animation display code unchanged)
        return (line_torque, line_pos, line_vel, label)

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
