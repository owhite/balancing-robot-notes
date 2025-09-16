# Sinusoidal Tracking Demo for Control Systems

## Why Sinusoidal Tracking Matters Universally

No matter what control law is used — PID, LQR, adaptive, sliding mode, even neural-network-based — they all share this requirement:

The closed-loop system must be able to track sinusoidal inputs with good fidelity, at least over the frequency range relevant to the task.

1. **Sinusoids are the building blocks**  
   - By Fourier theory, any signal can be decomposed into sinusoids.  
   - If your system can handle sine waves well, it can handle arbitrary signals.

2. **Defines Bandwidth**  
   - Sine tracking reveals your closed-loop bandwidth — the maximum frequency where accurate response is possible.  
   - Every controller type (PID, LQR, adaptive, etc.) is constrained by this.

3. **Reveals Gain & Phase Dynamics**  
   - Amplitude ratio and phase lag directly show stability margins.  
   - Works regardless of control law.

4. **Diagnostic & Comparable**  
   - Provides a common benchmark across control methods.  
   - Controllers can be compared directly based on their sine tracking quality.

---

## What Sinusoidal Tracking Demonstrates

1. **Smooth & repeatable input**  
   - A sine wave provides a predictable, continuous signal.  
   - Unlike random disturbances, it’s perfect for plotting and comparison.

2. **Tests both speed and accuracy**  
   - At low frequencies: checks static accuracy (does the motor follow the shape?).  
   - At high frequencies: reveals responsiveness and bandwidth (does the motor lag?).

3. **Measuring jitter**
   - Use in combination with jitterment
   - Test if continuous commands increase latency
   - Plots of reference vs. output make performance obvious.

4. **Direct relevance to balancing**  
   - Testing is representative of balance conditions
   - Balancing is about rejecting oscillatory disturbances.  
   - If the controller tracks sine inputs well, it is likely robust for balance recovery.

---

## Metrics for Tracking Quality

1. **Amplitude Ratio (Gain Error)**  
   - Compares size of output swing to input.  
   - Ideal = 1.0. Lower means sluggish, higher may mean overshoot or resonance.

2. **Phase Lag (Responsiveness)**  
   - Time delay between input and output.  
   - Expressed in degrees. Low-frequency lag should be near zero.

3. **Tracking Error (RMSE)**  
   - Root-mean-square error between reference and output.  
   - Lower values indicate better overall accuracy.

4. **Settling & Smoothness**  
   - Evaluates stability under disturbances.  
   - Output should remain smooth and resettle quickly if perturbed.

5. **Bandwidth (System Limit)**  
   - Frequency where output amplitude falls to ~70% of input and phase lag approaches 90°.  
   - Defines the effective responsiveness of the closed-loop system.

---

## Practical Thresholds

Generate hold_pos(t)=A⋅sin(2πft)
   - Start small: 𝐴 = 0.2 rad, f=0.5Hz.
   - Later sweep 𝑓 upward (1 Hz, 2 Hz, etc.).

Then evaluate
- **Gain**: ≥ 0.9 (output tracks at least 90% of input amplitude).  
- **Phase lag**: < 15° at frequencies relevant to balancing (e.g., ~1–2 Hz).  
- **RMSE**: < 5–10% of input amplitude.  

---

**Bottom line:** Sinusoidal tracking demonstrates both *responsiveness* and *accuracy*, providing a clear, quantitative measure of control system quality.
