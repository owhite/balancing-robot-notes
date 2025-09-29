# 🧩 The LQR Black-Box Workflow (Aligned with WBR)

## 1. **System Model (Black Box: Dynamics Generator)**
- **What it does:** Takes in your robot’s physical parameters (mass, inertia, wheel radius, geometry) and outputs a mathematical description of how the robot moves.
- **Outputs:** Two matrices:
  - A: how the robot’s **states** evolve naturally.
  - B: how the **motor inputs** affect those states.
- **WBR equivalent:**  
  - MATLAB: `M_f`, `nle_f`, `calculate_fx`, `calculate_fu`.  
  - C++: this is hidden — they just store the final gain matrices.  
- **You need to know:** States are like “tilt, tilt rate, velocity, yaw rate.” Inputs are like “torque left, torque right.”

---

## 2. **Linearization (Black Box: Linearizer)**
- **What it does:** Approximates the nonlinear robot dynamics near an equilibrium (standing up).  
- **Outputs:** Simplified linear equations that LQR can handle.
- **WBR equivalent:**  
  - MATLAB: `theta_eq(h)` and Jacobians (`calculateJacobian`).  
  - They linearize at different heights → which is why they do gain scheduling.
- **You need to know:** It’s like saying, “around upright, small tilts behave almost linearly, so let’s use that approximation.”

---

## 3. **Discretization (Black Box: Continuous → Digital Converter)**
- **What it does:** Converts continuous equations into discrete steps that match your sampling rate (e.g., every 8 ms).
- **Outputs:** Ad, Bd matrices for discrete-time control.
- **WBR equivalent:**  
  - MATLAB: `c2d(sys_c, Ts, 'zoh')`.  
- **You need to know:** Robots run on digital loops, not continuous math. This step bridges that gap.

---

## 4. **LQR Solver (Black Box: dlqr Function)**
- **What it does:** Takes your Ad, Bd plus cost weights Q, R, and solves an optimization problem (minimizes tilt/velocity errors while not using too much torque).
- **Inputs:**  
  - Q → penalties on states (balance, velocity, yaw).  
  - R → penalties on motor commands (don’t overdrive).  
- **Output:** Gain matrix K.
- **WBR equivalent:**  
  - MATLAB: `K_d = dlqr(Ad, Bd, Q_, R_)`.  
  - C++: those hardcoded `Ks.push_back(mat);`.
- **You need to know:** This is the true “black box.” You trust it to give you optimal gains, just like you trust `numpy.linalg.inv` to invert a matrix.

---

## 5. **Control Law (Black Box: Gain Application)**
- **What it does:** Applies the formula
  u = K · (x_d - x)
  where  
  - x = measured state (tilt, angular velocity, velocity, yaw rate).  
  - x_d = desired state (upright, target velocity, target yaw).  
  - u = motor torques.  
- **WBR equivalent:**  
  - C++: `computeInput(x_d, x);` in `VYBController.h`.  
- **You need to know:** This is just a matrix multiply — super cheap to compute on Teensy.

---

## 6. **Gain Scheduling (Black Box: Selector/Interpolator)**
- **What it does:** Picks or interpolates between different gain matrices depending on robot configuration (like height).
- **WBR equivalent:**  
  - C++: `computeGainK(h)` interpolates between entries in `Ks`.  
- **You need to know:** Instead of one K, you have a family of them for different heights. You just choose the right one at runtime.

---

## 7. **State Estimation (Black Box: Filter)**
- **What it does:** Cleans up noisy IMU + encoder data to give you reliable estimates of tilt, velocity, yaw rate.
- **WBR equivalent:**  
  - MATLAB: `EKF.m`.  
  - C++: `EKF Estimator(Pol);` or `CompenFilter`.  
- **You need to know:** You won’t feed raw gyro noise into LQR; you feed the filtered state estimate.

---

## 8. **Actuation (Black Box: Command Sender)**
- **What it does:** Converts motor torque commands into actual hardware signals.
- **WBR equivalent:**  
  - C++: `VYB_controller.sendControlCommand();` to the wheel servos.  
- **You need to know:** On your project, this will be: Teensy → MESC (via CAN/UART) → motor current.

---

# ✅ Summary of Black Boxes
1. **Dynamics Generator** → builds A, B.  
2. **Linearizer** → finds upright approximation.  
3. **Discretizer** → converts to digital time step.  
4. **LQR Solver** → outputs gain matrix K.  
5. **Control Law** → applies u = K(x_d - x).  
6. **Gain Scheduler** → switches/interpolates gains.  
7. **Estimator** → fuses IMU + encoders into clean states.  
8. **Actuation** → sends motor torque commands.  

---

👉 Here’s the good news: only **(1)–(4)** involve heavy math. And WBR already published working code for those. You can treat them like a **gain factory**: input robot specs → output gains.  
You then use **(5)–(8)** on your Teensy, which is mostly straightforward coding.
