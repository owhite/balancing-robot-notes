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
