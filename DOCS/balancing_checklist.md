# Balancing Robot Checklist (v2)

A staged roadmap for bring-up, testing, and validation.  
Each stage builds confidence before moving on.  

---

## Stage 0 — Comms & Timing (no control yet)  
**Goal:** Prove there’s no jitter and rates are correct.  

- Loop period (`dt` in ms) vs time  
  - Histogram of dt  
  - Min/mean/max  
  - **Also log ISR flag latency vs loop execution time**  
- CAN message rate (frames/s) per ID  
  - Rolling count by ID  
  - Utilization % (frames/s × avg bytes / bus bitrate)  
- UDP/Wi-Fi inter-arrival time (ms) & loss  
  - Sequence number for packet loss detection  
  - Latency estimate (TX vs RX timestamps)  
- Queue depths / buffers (TX, RX)  
- **CPU load % snapshot**  

---

## Stage 1 — Sensors Alive & Sane  
**Goal:** Confirm IMU/encoders are clean, oriented, and scaled.  

- IMU accel & gyro (3 axes) vs time  
- Wheel/encoder position & velocity  
- Battery voltage & current  
- Noise snapshot (PSD / spectrum of gyro Z or tilt rate)  
- **Gyro bias check:** log stationary gyro 10–30 s, compute mean offset  
- **Tilt sanity check:** tip robot ±10° and verify roll/pitch match within ~1–2°  
- **Encoder sign check:** forward tilt → positive wheel velocity  

---

## Stage 2 — State Estimation  
**Goal:** Validate state vector before closing the loop.  

- Define state vector explicitly:  
  ```
  theta (tilt), theta_dot (tilt rate), 
  x (wheel pos), x_dot (wheel vel)
  ```  
- Tilt angle (measured vs estimated) & tilt rate  
- **Innovation / residuals:**  
  - Definition: `innovation = measurement − prediction`  
  - Example:  
    - Gyro predicts tilt increased by 0.5°  
    - Accelerometer measures 0.3° tilt  
    - Innovation = −0.2°  
  - Small, zero-centered innovations → estimator is consistent  
  - Large/bias innovations → calibration or scaling issue  
- Confidence metrics (variance/covariance if available)  
- **Log gyro rate vs theta_dot estimate** → proves no double-integration drift  

---

## Stage 3 — Open-loop Actuator Tests (wheels off-ground)  
**Goal:** Verify commands map to motion.  

- Commanded motor effort vs measured wheel speed  
- Step response plots (speed vs time)  
- Current vs effort (torque constant check)  
- **Static torque test:** hold robot, command torque, check current scaling  
- **Sweep torque slowly:** verify linear velocity response  

---

## Stage 4 — Closed-loop Balancing at Fixed Height  
**Goal:** Bring it up gently and verify stability margins.  

- Strip chart: tilt angle, tilt rate, wheel velocity, (optionally) position drift  
- Control inputs `u` (per motor) vs time  
- Optional: Kx terms breakdown (`k_theta*theta`, `k_dtheta*theta_dot`, etc.)  
- Settle time & overshoot for small disturbances  
- **Tilt bias logging:** if robot balances at e.g. +2°, record offset  
- **Energy use:** motor current at steady balance vs disturbance recovery  

---

## Stage 5 — Robustness & Performance  
**Goal:** Quantify comfort margins.  

- Disturbance response (gentle pushes): angle, `u`, recovery time  
- Position creep/drift vs time at standstill  
- Motor/driver temperatures vs effort  
- Battery voltage sag vs thrust bursts  
- **Forward/backward velocity commands:** step test while balancing  
- **Long-duration run (10–15 min):** log tilt bias, wheel creep, motor temps  

---

## Stage 6 — Mobile Behavior (commanded motion)  
**Goal:** Verify the robot can balance *while moving intentionally*.  

- Commanded forward/backward velocity tracking  
  - Tilt bias vs commanded speed  
  - Wheel velocity vs command  
- Position tracking (drive forward X cm, stop)  
  - Overshoot, settle time  
- Turning tests (if differential steering): yaw rate vs command  
- Drift/logging: ensure tilt estimate remains bounded while in motion  
- Energy & thermal checks during continuous locomotion  

---

## Nice Quality-of-Life Plots (any stage)  
- Rolling stats panel: dt mean/std, packet loss %, CAN util %, CPU %  
- Event markers: log/annotate events (enable control, step, push)  
- Alarm flags: dt > threshold, loss > threshold, queue > N  
- **Saturation logging:** % time motor command at torque/voltage limit  
- **Loop execution time distribution**  

---

## Suggested Targets (ballpark)  
- Control loop dt: mean ~1.0–2.0 ms (500–1000 Hz), std dev < 0.1–0.2 ms  
- CAN command/telemetry: 250–500 Hz per critical topic, bus util < ~40%  
- UDP loss: < 0.5%, inter-arrival jitter < 5–10 ms typical  
- Tilt at standstill: < 0.5–1.0° RMS; recovery < 0.3–0.5 s after a small bump  
- Actuator clipping: rare in steady state; visible only on disturbances  
