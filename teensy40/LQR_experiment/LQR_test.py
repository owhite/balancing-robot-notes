#!/usr/bin/env python3
import sys, json, serial, math
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox, Button
import numpy as np
from scipy.linalg import solve_discrete_are

I_MAX = 30.0  # Amps, max current limit configured in ESC


def angle_diff(a, b):
    """Return minimal signed difference between two angles (wrap at 2π)."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def compute_discrete_lqr_gain(A_d, B_d, Q_term, R_term, B_term):
    """Compute discrete-time LQR gain matrix K."""
    Q = np.diag([Q_term, 1.0])
    R = np.array([[R_term]])
    P = solve_discrete_are(A_d, B_d, Q, R)
    K = np.linalg.inv(B_d.T @ P @ B_d + R) @ (B_d.T @ P @ A_d)

    print("\n=== DISCRETE-TIME LQR GAIN RECOMPUTATION ===")
    print("A_d =\n", A_d)
    print("B_d =\n", B_d)
    print(f"Qterm = {Q_term},  Rterm = {R_term},  Bterm = {B_term}")
    print("K_gain_discrete =", K)
    return K


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
    label.set_text("π/2 Time: --- ms")

    # Remove old crosshairs
    if hline is not None:
        hline.remove()
        hline = None
    if vline is not None:
        vline.remove()
        vline = None

    # Unpack TextBoxes
    tb_torque, tb_Rterm, tb_Qterm, tb_Bterm, tb_total = textboxes

    # Update parameters from UI
    params["torque"] = float(tb_torque.text)
    params["Rterm"] = float(tb_Rterm.text)
    params["Qterm"] = float(tb_Qterm.text)
    params["Bterm"] = float(tb_Bterm.text)
    params["total_ms"] = int(tb_total.text)

    with open(args.json_params, "w") as f:
        json.dump(params, f, indent=2)

    desired_torque_Nm = params["torque"]
    I = desired_torque_Nm / params["Bterm"]
    cmd = (I / I_MAX)

    LQR_data_path = params["LQR_data"]
    Q_term = params["Qterm"]
    R_term = params["Rterm"]
    B_term = params["Bterm"]

    # --- Load LQR data and compute K ---
    with open(LQR_data_path, "r") as f:
        data = json.load(f)

    A_d = np.array(data["A_matrix_discrete"])
    B_d = np.array(data["B_matrix_discrete"])
    K_disc = compute_discrete_lqr_gain(A_d, B_d, Q_term, R_term, B_term)

    # {'cmd': 'send', 'pulse_torque': 0.2, 'pulse_us': 250000, 'total_us': 3000000, 'user_Kp_term': 6.26, 'user_Kd_term': 0.6}

    print ("PARAMS: ", params)
    msg = {
        "cmd": "send",
        "pulse_torque": params["torque"],
        "total_us": params["total_ms"] * 1000, 
        "user_Kp_term": float(K_disc[0, 0]),
        "user_Kd_term": float(K_disc[0, 1])
    }

    print("TX:", msg)
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

    # --- UI controls ---
    axbox_torque = plt.axes([0.8, 0.90, 0.15, 0.05])
    axbox_Rterm  = plt.axes([0.8, 0.82, 0.15, 0.05])
    axbox_Qterm  = plt.axes([0.8, 0.74, 0.15, 0.05])
    axbox_Bterm  = plt.axes([0.8, 0.66, 0.15, 0.05])
    axbox_total  = plt.axes([0.8, 0.58, 0.15, 0.05])
    axbutton     = plt.axes([0.8, 0.46, 0.15, 0.07])

    tb_torque = TextBox(axbox_torque, "torque", initial=str(params["torque"]))
    tb_Rterm  = TextBox(axbox_Rterm, "R term", initial=str(params["Rterm"]))
    tb_Qterm  = TextBox(axbox_Qterm, "Q term", initial=str(params["Qterm"]))
    tb_Bterm  = TextBox(axbox_Bterm, "B term", initial=str(params["Bterm"]))
    tb_total  = TextBox(axbox_total, "total ms", initial=str(params["total_ms"]))
    button    = Button(axbutton, "Run")

    # --- Label display ---
    ax_label = plt.axes([0.8, 0.36, 0.15, 0.05])
    label = ax_label.text(0.0, 0.5, "arrive Time: --- ms", transform=ax_label.transAxes, fontsize=14)
    ax_label.axis("off")

    # --- Initialize state ---
    start_pos = None
    pi2_time_ms = None
    pi2_cross_pos = None
    hline = None
    vline = None
    axes_state = [start_pos, pi2_time_ms, pi2_cross_pos, hline, vline]
    textboxes = [tb_torque, tb_Rterm, tb_Qterm, tb_Bterm, tb_total]

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
