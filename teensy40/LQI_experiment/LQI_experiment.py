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

    # print("\n=== DISCRETE-TIME LQR GAIN RECOMPUTATION ===")
    # print("A_d =\n", A_d)
    # print("B_d =\n", B_d)
    # print(f"Qterm = {Q_term},  Rterm = {R_term},  Bterm = {B_term}")
    # print("K_gain_discrete =", K)
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

    # Remove old crosshairs
    if hline is not None:
        hline.remove()
        hline = None
    if vline is not None:
        vline.remove()
        vline = None

    # Unpack TextBoxes
    tb_torque, tb_total, tb_pulse = textboxes

    # Update parameters from UI
    params["torque"] = float(tb_torque.text)
    params["total_ms"] = int(tb_total.text)
    params["pulse_ms"] = int(tb_pulse.text)

    with open(args.json_params, "w") as f:
        json.dump(params, f, indent=2)

    desired_torque_Nm = params["torque"]
    LQR_data_path = params["LQR_data"]

    # --- Load LQR data and compute K ---
    with open(LQR_data_path, "r") as f:
        data = json.load(f)

    # K_disc = compute_discrete_lqr_gain(A_d, B_d, Q_term, R_term, B_term)

    # {'cmd': 'send', 'pulse_torque': 0.2, 'pulse_us': 250000, 'total_us': 3000000, 'user_Kp_term': 6.26, 'user_Kd_term': 0.6}

    msg = {
        "cmd": "pulse",
        "pulse_torque": params["torque"],
        "total_us": params["total_ms"] * 1000,
        "pulse_us": params["pulse_ms"] * 1000
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
    axbox_total  = plt.axes([0.8, 0.82, 0.15, 0.05])
    axbox_pulse  = plt.axes([0.8, 0.74, 0.15, 0.05])
    axbutton     = plt.axes([0.8, 0.46, 0.15, 0.07])

    tb_torque = TextBox(axbox_torque, "torque", initial=str(params["torque"]))
    tb_total  = TextBox(axbox_total, "total", initial=str(params["total_ms"]))
    tb_pulse  = TextBox(axbox_pulse, "pulse", initial=str(params["pulse_ms"]))
    button    = Button(axbutton, "Run")

    # --- Label display ---
    ax_label = plt.axes([0.8, 0.36, 0.15, 0.05])
    label = ax_label.text(0.0, 0.5, "---", transform=ax_label.transAxes, fontsize=14)
    ax_label.axis("off")

    # --- Initialize state ---
    start_pos = None
    pi2_time_ms = None
    pi2_cross_pos = None
    hline = None
    vline = None
    axes_state = [start_pos, pi2_time_ms, pi2_cross_pos, hline, vline]
    textboxes = [tb_torque, tb_total, tb_pulse]

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
                        # Take only the spin-down region (velocity > 0 and smooth decay)
                        t_decay = t[zero_idx:]
                        v_decay = vel[zero_idx:]

                        # Remove any zero or negative values before taking log
                        v_mask = v_decay > 1.0      # threshold to avoid log(0)
                        t_decay = t_decay[v_mask]
                        v_decay = v_decay[v_mask]

                        if len(t_decay) > 10:
                            # Fit exponential decay: v(t) = v0 * exp(-lambda * t)
                            logv = np.log(v_decay)
                            A = np.vstack([t_decay, np.ones(len(t_decay))]).T
                            slope, intercept = np.linalg.lstsq(A, logv, rcond=None)[0]
                            lam = -slope  # decay rate = b/J

                            print(f"✅ Estimated decay constant λ = {lam:.4f} s⁻¹ (so b/J = λ).")

                            # Optional: if J is known, compute b
                            J_est = 1.0e-4   # must be actual inertia [kg·m²]
                            b_est = lam * J_est
                            msg = f"λ={lam:.3f}s⁻¹\nb={b_est:.2e}\n(N·m·s/rad)"
                            print(f"✅ {msg}")
                            label.set_text(msg)

                        else:
                            msg = "no damping fit"
                            label.set_text(msg)
                            msg = "⚠️ Not enough valid samples for damping fit"
                            print(msg)
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
