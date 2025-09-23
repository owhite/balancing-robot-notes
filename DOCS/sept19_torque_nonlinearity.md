## ## Sept 19, 2035
# Torque Control in MESC: Issues & Solutions

## 1. Torque Control Concept
- In **Field-Oriented Control (FOC)**, torque ≈ *Kt × Iq*.  
- The MESC firmware implements **current (Iq) control**, so the **torque request = Iq request**.  
- In an *ideal ESC*, that would mean a perfectly linear “torque pipe”: Teensy commands torque → ESC outputs exact torque at the shaft.

---

## 2. Why Torque Control Isn’t Perfect in Practice
We identified several sources of **non-linearity, asymmetry, and inconsistency**:

1. **Deadtime asymmetry**  
   - +Iq vs –Iq doesn’t yield equal/opposite torque.  
   - Caused by MOSFET deadtime and voltage offsets.  

2. **Current sensor offset**  
   - DC offset in shunts or op-amps shifts measured torque.  
   - Leads to torque bias.  

3. **Kt nonlinearity (torque constant variation)**  
   - Torque per amp not constant (saturation, PWM distortion).  
   - “1 A” doesn’t always mean the same torque across the range.  

4. **Cogging torque (angle dependence)**  
   - Magnetic saliency causes torque valleys/peaks vs rotor angle.  
   - Plant feels “sticky” in some positions.  

5. **Static friction / stiction**  
   - Below a torque threshold, rotor doesn’t move at all.  

6. **Bus ripple / supply variation**  
   - Torque per amp varies with Vbus sag/ripple.  

7. **Latency in the loop** (Teensy → CAN → ESC → Teensy)  
   - Adds delay, reducing phase margin and repeatability.  

---

## 3. Consequences for Balancing Robot
- Same torque request can produce different results depending on angle, load, and history.  
- Leads to poor reproducibility, jitter, or random failures.  
- Teensy “brain” sees a noisy, nonlinear plant.  

---

## 4. Approaches to Solve / Mitigate

### A. Improve Torque Linearity (ESC-side fixes)
1. **Deadtime compensation**  
   - Adjust PWM voltage commands to cancel deadtime offsets.  

2. **Current sensor offset calibration**  
   - Measure offsets at startup and subtract.  

3. **Torque scaling (Kt gain)**  
   - Apply a correction factor to Iq request → torque mapping.  

4. **Cogging compensation (LUT)**  
   - Measure cogging profile in open loop.  
   - Store as lookup table indexed by electrical angle.  
   - Add/subtract in FOC: `Iq_req += cogging_table[idx]`.  

5. **Bus voltage normalization**  
   - Scale modulation commands by measured Vbus.  

6. **Anti-stiction bias**  
   - Add small bias/dither torque at near-zero speed.  

---

### B. Mask Nonlinearities with **Velocity Loop on ESC**
- Wrap a **velocity PID** around the torque controller (1–2 kHz).  
- Teensy commands *desired velocity*, ESC adjusts torque until achieved.  
- **Advantages:**  
  - Masks most asymmetries (deadtime, sensor offsets, Kt nonlinearity, supply ripple, stiction).  
  - Teensy sees a clean, linear velocity actuator.  
- **Limitations:**  
  - Cogging torque at very low speeds is still visible.  
  - Slightly less efficient (ESC may use more current to fight nonlinearities).  

---

### C. Hybrid Approach
- **Velocity loop on ESC** (masks most problems).  
- **Cogging compensation inside FOC** (handles the one issue velocity loop can’t mask).  
- Teensy runs higher-level control in terms of velocity/acceleration, with optional torque feedforward.  

---

### D. Diagnostic / Characterization Tools
- **Open loop mode** in MESC:  
  - Useful for characterizing cogging torque and asymmetries without observer correction.  

- **Practical test procedure (A1, A2, A3 profiles):**  
  1. Lock motor at a specific angle → apply torque → measure acceleration.  
  2. Repeat at different angles.  
  3. Repeat with opposite torque sign.  
  - If results differ → shows cogging, asymmetry, or scaling error.  

---

## 5. Summary Table

| Issue                        | ESC Torque Mode | ESC Velocity Loop | Cogging LUT |
|------------------------------|-----------------|------------------|-------------|
| Deadtime asymmetry           | Visible         | Masked           | – |
| Current sensor offset        | Visible         | Masked           | – |
| Kt nonlinearity              | Visible         | Masked           | – |
| Cogging torque               | Visible         | Partially masked | ✅ Fixed |
| Static friction / stiction   | Visible         | Masked           | – |
| Bus ripple / supply variation| Visible         | Masked           | – |
| Latency (Teensy↔ESC)         | Always present  | Reduced impact   | – |

---

## 📌 Big Picture Plan
1. **Short-term:** torque control with improved velocity estimate (what you’re already testing).  
2. **Medium-term:** add a **velocity loop inside MESC** → cleaner plant for Teensy.  
3. **Structural fix:** add **cogging compensation table** inside MESC’s FOC loop.  
4. **Optional refinements:** deadtime comp, Kt scaling, bus normalization for efficiency.  
