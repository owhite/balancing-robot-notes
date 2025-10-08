#!/usr/bin/env python3
import sys, json, serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox, Button
import numpy as np
from scipy import signal
from scipy.linalg import solve_continuous_are

PARAM_FILE = "params.json"

# ---------------- Load parameters ----------------
def load_params():
    try:
        with open(PARAM_FILE, "r") as f:
            return json.load(f)
    except:
        return {
            "amps": 1.0,
            "max_angle": 0.4,
            "duration_ms": 5000,
            "bit_time_ms": 30,
            "I_MAX": 30.0,
            "port": "/dev/cu.usbmodem178888901"
        }

params = load_params()
port = params["port"]
ser = serial.Serial(port, 115200, timeout=0.1)
burst_data = None

# ---------------- Plot setup ----------------
fig, (ax_i, ax_pos, ax_vel) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
plt.subplots_adjust(left=0.1, right=0.75, top=0.92, bottom=0.1)

line_i, = ax_i.plot([], [], 'r', label="Command Current")
ax_i.set_title("PRBS Input Current vs Time")
ax_i.set_ylabel("Command (A)")
ax_i.legend(); ax_i.grid(True)

line_pos, = ax_pos.plot([], [], 'g', label="Position")
ax_pos.set_title("Pendulum Position vs Time")
ax_pos.set_ylabel("Position (rad)")
ax_pos.legend(); ax_pos.grid(True)

line_vel, = ax_vel.plot([], [], 'b', label="Velocity")
ax_vel.set_title("Pendulum Velocity vs Time")
ax_vel.set_xlabel("t (ms)")
ax_vel.set_ylabel("Velocity (rad/s)")
ax_vel.legend(); ax_vel.grid(True)

# ---------------- UI Controls ----------------
axbox_amps      = plt.axes([0.8, 0.90, 0.15, 0.05])
axbox_angle     = plt.axes([0.8, 0.82, 0.15, 0.05])
axbox_duration  = plt.axes([0.8, 0.74, 0.15, 0.05])
axbox_bit       = plt.axes([0.8, 0.66, 0.15, 0.05])
axbutton        = plt.axes([0.8, 0.54, 0.15, 0.07])

tb_amps = TextBox(axbox_amps, "", initial=str(params["amps"]))
axbox_amps.set_title("amps (A)", fontsize=10)
tb_angle = TextBox(axbox_angle, "", initial=str(params["max_angle"]))
axbox_angle.set_title("max_angle (rad)", fontsize=10)
tb_duration = TextBox(axbox_duration, "", initial=str(params["duration_ms"]))
axbox_duration.set_title("duration_ms", fontsize=10)
tb_bit = TextBox(axbox_bit, "", initial=str(params["bit_time_ms"]))
axbox_bit.set_title("bit_time_ms", fontsize=10)
button = Button(axbutton, "Run")

# ---------------- Button handler ----------------
def save_and_run(event):
    params["amps"] = float(tb_amps.text)
    params["max_angle"] = float(tb_angle.text)
    params["duration_ms"] = int(tb_duration.text)
    params["bit_time_ms"] = int(tb_bit.text)
    with open(PARAM_FILE, "w") as f:
        json.dump(params, f, indent=2)

    msg = {
        "cmd": "send", "mode": "PRBS",
        "amp_command": params["amps"],
        "max_angle": params["max_angle"],
        "duration_ms": params["duration_ms"],
        "bit_time_ms": params["bit_time_ms"]
    }
    print(f"TX: {msg}")
    ser.write((json.dumps(msg) + "\n").encode("utf-8"))

button.on_clicked(save_and_run)

# ---------------- Update loop ----------------
def update(frame):
    global burst_data

    while ser.in_waiting:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line: continue
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            if line.startswith("{") and "samples" in line:
                buf = line
                while True:
                    chunk = ser.readline().decode("utf-8", errors="ignore")
                    if not chunk: break
                    buf += chunk
                    try:
                        data = json.loads(buf); break
                    except json.JSONDecodeError:
                        continue
                if "samples" in data:
                    burst_data = data["samples"]
                    print(f"Received {len(burst_data)} samples")
            continue

    if burst_data is not None:
        t0 = burst_data[0]["t"]
        t_vals = np.array([(s["t"] - t0) * 1e-3 for s in burst_data])  # µs → ms
        i_vals = np.array([s["torque"] * params["I_MAX"] for s in burst_data])
        pos_vals = np.array([s["pos"] for s in burst_data])
        vel_vals = np.array([s["vel"] for s in burst_data])

        # ----- update plots -----
        line_i.set_data(t_vals, i_vals)
        line_pos.set_data(t_vals, pos_vals)
        line_vel.set_data(t_vals, vel_vals)
        for ax, vals in [(ax_i, i_vals), (ax_pos, pos_vals), (ax_vel, vel_vals)]:
            ymin, ymax = min(vals), max(vals)
            ax.set_xlim(min(t_vals), max(t_vals))
            ax.set_ylim(ymin - 0.1*(ymax-ymin), ymax + 0.1*(ymax-ymin))

        # ----- Frequency response -----
        dt = (t_vals[1]-t_vals[0]) / 1000.0  # s
        fs = 1.0 / dt
        u = i_vals - np.mean(i_vals)
        y = vel_vals - np.mean(vel_vals)
        f, Pxy = signal.csd(y,u,fs=fs,nperseg=512)
        _, Pxx = signal.welch(u,fs=fs,nperseg=512)
        H = Pxy/Pxx
        mag, phase = 20*np.log10(np.abs(H)), np.angle(H,deg=True)

        fig_bode,(ax_mag,ax_phase)=plt.subplots(2,1,figsize=(7,6),sharex=True)
        ax_mag.semilogx(f,mag,'b'); ax_mag.set_ylabel("Magnitude (dB)")
        ax_mag.set_title("Bode Plot (Current → Velocity)")
        ax_mag.grid(True,which='both')
        ax_phase.semilogx(f,phase,'r'); ax_phase.set_xlabel("Frequency (Hz)")
        ax_phase.set_ylabel("Phase (deg)"); ax_phase.grid(True,which='both')

        # ----- Impulse response -----
        h_time = np.fft.ifft(H)
        fig_imp, ax_imp = plt.subplots(figsize=(7,3))
        ax_imp.plot(np.real(h_time)); ax_imp.set_title("Impulse Response")
        ax_imp.set_xlabel("Samples"); ax_imp.grid(True)

        # ----- Discrete-time transfer function estimation -----
        print("\nEstimating discrete-time 2nd order transfer function...")
        n_imp = 300
        h = np.real(h_time[:n_imp])
        yk = h[2:]
        Phi = np.column_stack([-h[1:-1], -h[:-2]])
        a_coeffs, *_ = np.linalg.lstsq(Phi, yk, rcond=None)
        a1, a2 = a_coeffs
        b0 = h[0]
        num, den = [b0], [1.0, a1, a2]

        poles = np.roots(den)
        if np.any(np.abs(poles) >= 1):
            print("⚠️ Unstable discrete poles detected — reflecting inside unit circle.")
            poles = np.where(np.abs(poles)>=1, 1/np.conj(poles), poles)
            den = np.poly(poles).real

        print("Estimated TF coefficients:")
        print("  Numerator:", np.round(num,5))
        print("  Denominator:", np.round(den,5))

        sysd = signal.dlti(num, den)
        t_step, y_step = signal.dstep(sysd)
        t_step, y_step = np.squeeze(t_step)*dt, np.squeeze(y_step)
        fig_step, ax_step = plt.subplots(figsize=(7,3))
        ax_step.plot(t_step, y_step)
        ax_step.set_title("Step Response (Discrete-Time Fitted Model)")
        ax_step.set_xlabel("Time (s)"); ax_step.set_ylabel("Velocity (rad/s)")
        ax_step.grid(True)

        # ----- Approximate continuous-time equivalent using bilinear transform -----
        print("\nConverting to continuous-time via bilinear transform...")
        # Get discrete-time poles/zeros/gain
        z, p, k = signal.tf2zpk(num, den)

        # Use bilinear transform to map z-plane → s-plane
        # (inverse of Tustin’s method)
        fs = 1.0 / dt
        p_cont = 2 * fs * (p - 1) / (p + 1)
        z_cont = 2 * fs * (z - 1) / (z + 1) if len(z) else np.array([])
        k_cont = k * np.prod(1 - z) / np.prod(1 - p)

        numc, denc = signal.zpk2tf(z_cont, p_cont, k_cont)

        print("Continuous-time approximate model:")
        print("  num =", np.round(numc, 5))
        print("  den =", np.round(denc, 5))

        # Rebuild continuous-time system from the already-converted coefficients
        sysc = signal.TransferFunction(numc, denc)

        # ----- Estimate inertia (J) and damping (b) -----
        J_est, b_est = None, None
        if len(denc) == 2:
            # First-order system: G(s) = (1/J)/(s + b/J)
            J_est = 1.0 / numc[0]
            b_est = denc[1] / numc[0]
        elif len(denc) == 3:
            # Second-order system: G(s) = (1/J)/(s^2 + (b/J)s + k/J)
            J_est = 1.0 / numc[0]
            b_est = denc[1] / numc[0]
            k_est = denc[2] / numc[0]
            print(f"\nEstimated parameters (2nd-order fit):")
            print(f"  Inertia J ≈ {J_est:.4e}")
            print(f"  Damping b ≈ {b_est:.4e}")
            print(f"  Stiffness k ≈ {k_est:.4e}")
        else:
            print("⚠️ Model order too high — skipping parameter extraction.")

        # Provide fallback if extraction failed
        if J_est is None or b_est is None:
            print("⚠️ Using fallback parameters for LQR design.")
            J_est, b_est = 0.01, 0.1  # safe defaults

        # ----- LQR design -----
        print("\nDesigning LQR controller...")
        A = np.array([[0.0, 1.0],
                      [-k_est / J_est, -b_est / J_est]])   # include stiffness!
        B = np.array([[0.0],
                      [1.0 / J_est]])
        C = np.array([[1.0, 0.0]])   # output = angle
        D = np.array([[0.0]])

        Q = np.diag([10, 1])   # state weighting
        R = np.array([[0.1]])  # control weighting
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ P
        print("\nLQR gain matrix K =", np.round(K,5))

        # ----- Closed-loop simulation -----
        Acl = A - B @ K
        sys_cl = signal.StateSpace(Acl, B, C, D)
        t_cl, y_cl = signal.step(sys_cl)
        fig_cl, ax_cl = plt.subplots(figsize=(7,3))
        ax_cl.plot(t_cl, y_cl)
        ax_cl.set_title("Closed-Loop Step Response (LQR Controlled)")
        ax_cl.set_xlabel("Time (s)")
        ax_cl.set_ylabel("Angle (rad)")
        ax_cl.grid(True)

        # ----- Continuous-time equivalent summary -----
        print("\n✅ Final continuous-time model and LQR summary:")
        print("  G(s) = ", np.round(numc, 5), "/", np.round(denc, 5))
        print(f"  LQR gains K = {np.round(K,5)}")

        # ----- Closed-loop validation (compare open vs closed loop) -----

        # Open-loop step (unit torque); plot in microradians so it’s visible
        t_ol, y_ol = signal.step(sysc, T=np.linspace(0, 2.0, 2000))  # 2 s window; adjust as needed
        y_ol_urad = y_ol * 1e6

        Acl = A - B @ K
        sys_cl = signal.StateSpace(Acl, B, C, D)
        t_cl, y_cl = signal.step(sys_cl)
        y_cl_urad = y_cl * 1e6

        fig_cmp, ax_cmp = plt.subplots(figsize=(7,3))
        ax_cmp.plot(t_ol, y_ol_urad, 'r--', lw=2, alpha=0.9, label="Open-loop (µrad)")
        ax_cmp.plot(t_cl, y_cl_urad, 'b',   lw=2, alpha=0.9, label="Closed-loop (µrad)")
        ax_cmp.set_xlim(0, max(t_cl))        # or a shorter window to zoom
        ax_cmp.set_title("Step Response: Open vs. LQR Closed Loop")
        ax_cmp.set_xlabel("Time (s)")
        ax_cmp.set_ylabel("Angle (µrad)")
        ax_cmp.legend(); ax_cmp.grid(True)

        print("numc:", numc)
        print("denc:", denc)
        print("DC gain:", numc[-1]/denc[-1])

        # ----- Save results -----
        results = {
            "numc": numc.tolist(),
            "denc": denc.tolist(),
            "J_est": float(J_est),
            "b_est": float(b_est),
            "K": K.tolist()
        }
        with open("tf_results.json", "w") as f:
            json.dump(results, f, indent=2)
        print("\nSaved model and LQR data to tf_results.json")

        plt.show(block=False)
        burst_data = None
    return (line_i, line_pos, line_vel)

ani=animation.FuncAnimation(fig,update,interval=100,blit=False,cache_frame_data=False)
plt.show()
