#!/usr/bin/env python3
"""
LQR Design, Simulation, and Export for Teensy Firmware Integration
------------------------------------------------------------------

- Loads system matrices (A, B) from pendulum_LQR_data.json
- Computes continuous-time LQR gain K
- Simulates closed-loop continuous and discrete-time responses
- Exports controller gains, matrices, and metadata for Teensy use
"""

import numpy as np
import json, sys, os
from scipy.linalg import solve_continuous_are, expm
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

def design_lqr(A, B, Q, R):
    """Solve the continuous-time LQR problem."""
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ (B.T @ P)
    eigvals, _ = np.linalg.eig(A - B @ K)
    return K, P, eigvals

def simulate_response(A, B, K, x0, t_final=3.0):
    """Simulate x_dot = (A - B K) x."""
    Acl = A - B @ K
    def f(t, x):
        return (Acl @ x).flatten()
    t_eval = np.linspace(0, t_final, 1000)
    sol = solve_ivp(f, [0, t_final], x0, t_eval=t_eval, method="LSODA")
    x = sol.y.T
    u = np.array([-(K @ xi.reshape(-1,1)).item() for xi in x])
    return sol.t, x, u

def discretize_system(A, B, Ts):
    """Compute discrete-time equivalents for Teensy control loop."""
    # expm block method: [A B; 0 0]
    n = A.shape[0]
    M = np.zeros((n+1, n+1))
    M[:n,:n] = A
    M[:n,n:] = B
    Md = expm(M * Ts)
    Ad = Md[:n,:n]
    Bd = Md[:n,n:]
    return Ad, Bd

def main():
    if len(sys.argv) < 2:
        print("Usage: python lqr_simulate_and_export.py pendulum_LQR_data.json [sample_period_s]")
        sys.exit(1)

    input_path = sys.argv[1]
    Ts = float(sys.argv[2]) if len(sys.argv) > 2 else 0.001  # default 1 kHz loop

    with open(input_path, "r") as f:
        data = json.load(f)

    # Extract system and motor parameters
    A = np.array(data["A_matrix"])
    B = np.array(data["B_matrix"])
    m = data["mass_kg"]
    I = data["moment_of_inertia_kg_m2"]
    r = data["r_m"]
    motor = data.get("motor_params", {})
    Kt = motor.get("Kt", None)

    print("\n📘 Loaded system parameters:")
    print(f"  Mass m = {m:.6f} kg")
    print(f"  Moment of inertia I = {I:.6e} kg·m²")
    print(f"  Lever arm r = {r:.6f} m")
    if Kt:
        print(f"  Motor torque constant Kt = {Kt:.6f} N·m/A")

    # Continuous LQR
    Q = np.diag([10.0, 1.0])
    # R = np.array([[1.0]])
    R = np.array([[0.1]])
    K, P, eigvals = design_lqr(A, B, Q, R)
    print("\n📊 LQR design summary:")
    print(f"  Q = {Q}")
    print(f"  R = {R}")
    print(f"  Gain K = {K}")
    print(f"  Closed-loop eigenvalues = {eigvals}")

    real_parts = np.real(eigvals)
    tau_dom = -1.0 / np.max(real_parts)   # slowest pole
    tau_fast = -1.0 / np.min(real_parts)  # fastest pole
    print(f"  ⇒ Dominant time constant ≈ {tau_dom:.3f} s")
    print(f"  ⇒ Fastest pole ≈ {tau_fast:.3f} s")

    # Simulate continuous-time response
    x0 = np.array([0.1, 0.0])  # initial angle 0.1 rad (~5.7°)
    t, x, u = simulate_response(A, B, K, x0)

    # Discretize for Teensy firmware loop
    Ad, Bd = discretize_system(A, B, Ts)
    Kd = K  # identical gain used; future improvement: discrete LQR if needed

    # Plot
    plt.figure(figsize=(7,4))
    plt.plot(t, x[:,0], label="Angle θ (rad)")
    plt.plot(t, x[:,1], label="Angular velocity θ̇ (rad/s)")
    plt.xlabel("Time [s]")
    plt.ylabel("State")
    plt.title("LQR Closed-Loop Response (Continuous-Time)")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.show()

    # Export JSON
    out_data = {
        "metadata": {
            "description": "LQR controller and simulation results for Teensy firmware",
            "units": {
                "angle": "radians",
                "angular_velocity": "radians_per_second",
                "torque": "newton_meters",
                "motor_current": "amperes (if computed via torque / Kt)"
            },
            "sample_period_s": Ts,
            "note": "Teensy control assumes u = -K @ x (row vector gain)"
        },
        "K_gain": K.tolist(),
        "closed_loop_eigenvalues": [[float(ev.real), float(ev.imag)] for ev in eigvals],
        "A_matrix": A.tolist(),
        "B_matrix": B.tolist(),
        "A_discrete": Ad.tolist(),
        "B_discrete": Bd.tolist(),
        "K_discrete": Kd.tolist(),
        "time_s": t.tolist(),
        "angle_rad": x[:,0].tolist(),
        "angular_velocity_rad_s": x[:,1].tolist(),
        "control_torque_Nm": u.tolist(),
        "Q": Q.tolist(),
        "R": R.tolist(),
        "mass_kg": m,
        "moment_of_inertia_kg_m2": I,
        "lever_arm_m": r,
        "Kt_Nm_per_A": Kt,
        "dominant_time_constant_s": tau_dom
    }

    out_path = os.path.join(os.path.dirname(input_path), "lqr_sim_output.json")
    with open(out_path, "w") as f:
        json.dump(out_data, f, indent=2, ensure_ascii=False)
    print(f"\n✅ Exported enhanced LQR data and simulation results to: {out_path}")
    print(f"✅ Controller gain K saved for firmware use: {K.tolist()}")
    print("✅   NOTE: K was recomputed, did not use previous")
    print(f"✅ Discrete matrices (A_d, B_d) for sample period {Ts*1000:.1f} ms")

if __name__ == "__main__":
    main()
