#!/usr/bin/env python3
import numpy as np
import json
from scipy.linalg import solve_continuous_are
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import sys, os

def design_lqr(A, B, Q, R):
    """Solve the continuous-time LQR problem."""
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ (B.T @ P)
    eigvals, _ = np.linalg.eig(A - B @ K)
    return K, P, eigvals

def simulate_response(A, B, K, x0, t_final=3.0):
    """Simulate x_dot = (A - B K) x using a stiff-stable solver."""
    Acl = A - B @ K
    def f(t, x):
        return (Acl @ x).flatten()
    t_eval = np.linspace(0, t_final, 1000)
    sol = solve_ivp(f, [0, t_final], x0, t_eval=t_eval, method="LSODA")
    x = sol.y.T
    u = np.array([-(K @ xi.reshape(-1,1)).item() for xi in x])
    return sol.t, x, u

def main():
    if len(sys.argv) < 2:
        print("Usage: python lqr_simulate_and_export.py pendulum_LQR_data.json")
        sys.exit(1)

    input_path = sys.argv[1]
    with open(input_path, "r") as f:
        data = json.load(f)

    # Extract system matrices
    A = np.array(data["A_matrix"])
    B = np.array(data["B_matrix"])
    m = data["mass_kg"]
    I = data["moment_of_inertia_kg_m2"]
    r = data["r_m"]

    # LQR weights
    Q = np.diag([10.0, 1.0])
    R = np.array([[1.0]])  # ← keep this

    # Design controller
    K, P, eigvals = design_lqr(A, B, Q, R)
    print("\n📊 LQR design summary:")
    print("  Q =", Q)
    print("  R =", R)
    print("  Gain K =", K)
    print("  Closed-loop eigenvalues =", eigvals)

    # Simulate closed-loop response
    x0 = np.array([0.1, 0.0])  # initial angle 0.1 rad (~5.7°)
    t, x, u = simulate_response(A, B, K, x0, t_final=3.0)

    # Plot
    plt.figure(figsize=(7,4))
    plt.plot(t, x[:,0], label="Angle θ (rad)")
    plt.plot(t, x[:,1], label="Angular velocity θ̇ (rad/s)")
    plt.xlabel("Time [s]")
    plt.ylabel("State")
    plt.title("LQR Closed-Loop Response (Linearized Model)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Export results
    out_data = {
        "K_gain": K.tolist(),
        "closed_loop_eigenvalues": [complex(ev).real for ev in eigvals],
        "A_matrix": A.tolist(),
        "B_matrix": B.tolist(),
        "time_s": t.tolist(),
        "angle_rad": x[:,0].tolist(),
        "angular_velocity_rad_s": x[:,1].tolist(),
        "control_torque": u.tolist(),
        "Q": Q.tolist(),
        "R": R.tolist(),
        "mass_kg": m,
        "moment_of_inertia_kg_m2": I,
        "lever_arm_m": r
    }

    out_path = os.path.join(os.path.dirname(input_path), "lqr_sim_output.json")
    with open(out_path, "w") as f:
        json.dump(out_data, f, indent=2)
    print(f"\n✅ Exported LQR data and simulation results to: {out_path}")

if __name__ == "__main__":
    main()
