## Sept 18, 2025
# Motor Controller System Issues Summary
Or, a list of things that can go wrong and must be ruled out. 

## 1. Torque Request → Phase Current Mapping
- Torque command may not map cleanly to actual **Iq current**, and therefore not to torque.  
- Causes: ESC nonlinearities, sensor noise, uncalibrated Kt, firmware cutoffs.  
- Result: unpredictable motor response, PID “chasing a moving target.”

---

## 2. Non-First-Order Plant (Wheel + Body)
- A motor alone looks roughly like a first-order system (current → torque → acceleration).  
- Once attached to a wheel and body:
  - The system becomes **at least second-order** (inertia + acceleration dynamics).  
  - If balancing (like a robot), it also includes **unstable poles**.  
- PID is designed for first-order plants → it won’t naturally stabilize this higher-order, unstable system.

---

## 3. Classic PID Limitations
- PID assumes proportional output to input, but your plant has:
  - Delays, inertia, and multiple energy storage elements.  
  - Nonlinear torque ↔ current mapping at low speeds.  
- This mismatch explains why endless PID tuning never yields stability.  

---

## 4. Missing Feedforward
- Without feedforward, PID has to do all the work reactively.  
- A feedforward torque term (e.g. to cancel gravity, rolling resistance, or expected acceleration) reduces error before feedback kicks in.  
- Advanced controllers like **moteus** rely heavily on feedforward for stability.

---

## 5. Second-Order Behavior
- Second order dynamics:  
  ```
  θ¨ = (1/J) * (τ - τdisturbance)
  ```
- Properties:  
  - Overshoot, ringing, and instability if damping is insufficient.  
  - PID can only add limited damping, and poorly if torque mapping is wrong.  
- Without explicit handling of second-order effects, the system oscillates instead of settling.

---

## 6. Other Likely Factors
- **Sensor limits**: encoder resolution, IMU noise → unstable estimates.  
- **Back-EMF & voltage saturation**: at higher speeds → torque per amp falls.  
- **Friction/cogging**: at low speed → deadzone effects that PID can’t handle smoothly.  
- **Task split**: if inner current loop isn’t fast enough, outer loop commands don’t track.

---

## ✅ One-Sentence Summary
Your system doesn’t follow control law because it’s **a nonlinear, second-order plant (or higher), driven by a torque–current mapping that isn’t accurate**, so PID designed for a linear first-order system will struggle. Higher-level control (feedforward, LQR, gain scheduling) is needed to stabilize it.

--- Some other items to consider:

- This video is an ESP32 with github code: [LINK](https://www.youtube.com/watch?v=rpD8mo0Jbuc)
- They aren’t using simple PID.
- With LKF state estimates, they probably use state feedback (like LQR) or PID + feedforward, which can explicitly stabilize second-order dynamics.
- Their robot balances smoothly, while yours oscillates or fails to converge.

# Experiment Plan: Diagnosing Plant Order in Motor Controller System

##  **Diagnose order of the plant**  
1. **Steps**  
   - Apply a step input (setpoint far from actual position) and observe response.  
   - A first-order system shows exponential approach.  
   - A second-order system shows overshoot/ringing.

2. **Burst logging**  
   - Avoid serial output during motion to prevent bottlenecks.  
   - Log data to RAM and dump it only after the test completes.

3. **Setpoint offset**  
   - Start far from the setpoint to create a **clear transient**.  
   - This highlights overshoot, ringing, or divergence.

---

## 📊 What to Record
- **Commanded setpoint** (position or velocity).  
- **Measured position** (encoder).  
- **Velocity estimate** (if available).  
- **Control effort** (torque command or current request).  
- **Phase current feedback** (if available).  
- **Bus voltage / battery voltage** (to catch sag effects).  
- **Error** = setpoint − position.  
- **Optional:** IMU tilt (if balancing robot).  

---

## 🔍 What to Look For
- **First-order behavior:** smooth exponential decay to setpoint, no overshoot.  
- **Second-order behavior:** overshoot, ringing, oscillation before settling.  
- **Nonlinearities:** dead zones, saturation (flat response until torque jumps), current clipping.  
- **Instability:** divergence instead of convergence.  

---

## ⚡ Refinement Ideas
- Run with different step sizes (small vs. large) → nonlinearities will show up if response shape changes.  
- Try with and without load (wheel free vs. on ground).  
- Include a **timestamp or tick counter** in logs to reconstruct dynamics.  

---

## ✅ Conclusion
This experiment will let you see clearly why the plant doesn’t behave like first-order. By analyzing step responses, you can confirm whether the system is **second-order (or higher)** and uncover nonlinearities that break PID control assumptions.
