# Measuring Phase, d-axis, and q-axis Inductance of a BLDC Motor
*(Step-response method with bench supply and oscilloscope)*

## Goal
Measure the **phase-to-phase inductance** of a BLDC motor using a current step response, then extract **Ld** and **Lq** by repeating the measurement versus rotor angle.

This procedure was applied to a **GBM5208-200T (12N14P, wye)** motor and produced time constants ranging from **600 µs to 1000 µs** depending on rotor position.

---

## Required hardware
- BLDC motor (three leads)
- Bench power supply (5 V used here)
- Series resistor: **1 Ω, ≥10 W**
- Manual switch 
- Oscilloscope with single-shot capture
- Test leads

---

## Physical setup

### Wiring
1. Choose two motor phases (A and B). Leave the third phase floating.
2. Wire in series:

   **Supply + → switch → 1 Ω resistor → phase A → phase B → supply −**

This forms a simple series **R–L** circuit.

### Oscilloscope
3. Connect the scope probe **across the 1 Ω resistor**.
   - This voltage directly represents current:
     - **1 V = 1 A**

(Optional: second channel across the motor terminals.)

---

## Electrical preparation

### Measure resistance
4. Measure phase-to-phase DC resistance with a DMM (rotor stationary).

Example (GBM5208-200T):
- Phase-to-phase resistance: **≈17 Ω**
- Series resistor: **1 Ω**

5. Compute total resistance:
```
R_total ≈ 17 Ω + 1 Ω = 18 Ω
```

### Choose supply voltage
6. Use a low voltage to limit current and torque.
   - **5 V** was used

Expected steady current:
```
I = V / R_total ≈ 5 / 18 ≈ 0.28 A
```

Expected scope plateau:
```
V_shunt ≈ 0.28 V
```

---

## Capturing the step response

7. Configure the scope:
   - Single-shot mode
   - Trigger on rising edge of shunt voltage
   - Timebase so the rise is clearly visible (hundreds of µs/div)

8. Close the switch to apply the voltage step.

You should see an exponential rise in the shunt voltage.

---

## Extracting inductance

### Use the 63.2% time-constant method
9. Measure the final shunt voltage plateau \(V_final\).

10. Compute the 63.2% level:
```
V_63 = 0.632 × V_final
```

11. Measure the time from the step start to when the trace crosses \(V_63\).
    This time is the RL time constant **τ**.

- Put tape on 60 degrees of the motor
- Mark ~8 spots
- Putting 1V on the motor makes it jump, so clamp to the table to hold position
- Take 8 measurements

12. Compute phase-to-phase inductance:
```
L_pp = τ × R_total
```
use min and max values from the 8 measurements

---

## Measuring Ld and Lq (saliency)

The motor is **12N14P**:
- 14 poles → **7 pole pairs**
- Inductance varies with rotor angle

### Practical method
13. Repeat the step-response measurement while holding the rotor at different positions.
14. Record the minimum and maximum time constants observed.

For the **GBM5208-200T**:
- τ_min ≈ **600 µs**
- τ_max ≈ **1000 µs**

15. Compute phase-to-phase inductances:
```
L_d,pp ≈ τ_min × R_total ≈ 600 µs × 18 Ω ≈ 10.8 mH
L_q,pp ≈ τ_max × R_total ≈ 1000 µs × 18 Ω ≈ 18.0 mH
```

Assign:
- **Ld ≈ minimum inductance**
- **Lq ≈ maximum inductance**

---

## Convert to per-phase inductance (wye connection)

For a symmetric wye motor:
```
L_phase ≈ L_pp / 2
```

So:
```
Ld ≈ 10.8 mH / 2 ≈ 5.4 mH
Lq ≈ 18.0 mH / 2 ≈ 9.0 mH
```

Saliency ratio:
```
Lq / Ld ≈ 1.67
```

---

## Notes for repeatability
- Keep voltage low to avoid rotor motion and heating.
- Ignore initial switching noise; use the exponential envelope.
- Use short leads and, if needed, limit scope bandwidth (~20 MHz).
- Report inductance as a range if rotor angle is uncontrolled.

---

# Measuring Phase, d-axis, and q-axis Inductance of a BLDC Motor
*(Step-response method with bench supply and oscilloscope)*

## Goal
Measure the **phase-to-phase inductance** of a BLDC motor using a current step response, then extract **Ld** and **Lq** by repeating the measurement versus rotor angle.

This procedure was applied to a **GBM5208-200T (12N14P, wye)** motor and produced time constants ranging from **600 µs to 1000 µs** depending on rotor position.

---

## Required hardware
- BLDC motor (three leads)
- Bench power supply (5 V used here)
- Series resistor: **1 Ω, ≥10 W**
- Manual switch 
- Oscilloscope with single-shot capture
- Test leads

---

## Physical setup

### Wiring
1. Choose two motor phases (A and B). Leave the third phase floating.
2. Wire in series:

   **Supply + → switch → 1 Ω resistor → phase A → phase B → supply −**

This forms a simple series **R–L** circuit.

### Oscilloscope
3. Connect the scope probe **across the 1 Ω resistor**.
   - This voltage directly represents current:
     - **1 V = 1 A**

(Optional: second channel across the motor terminals.)

---

## Electrical preparation

### Measure resistance
4. Measure phase-to-phase DC resistance with a DMM (rotor stationary).

Example (GBM5208-200T):
- Phase-to-phase resistance: **≈17 Ω**
- Series resistor: **1 Ω**

5. Compute total resistance:
```
R_total ≈ 17 Ω + 1 Ω = 18 Ω
```

### Choose supply voltage
6. Use a low voltage to limit current and torque.
   - **5 V** was used

Expected steady current:
```
I = V / R_total ≈ 5 / 18 ≈ 0.28 A
```

Expected scope plateau:
```
V_shunt ≈ 0.28 V
```

---

## Capturing the step response

7. Configure the scope:
   - Single-shot mode
   - Trigger on rising edge of shunt voltage
   - Timebase so the rise is clearly visible (hundreds of µs/div)

8. Close the switch to apply the voltage step.

You should see an exponential rise in the shunt voltage.

---

## Extracting inductance

### Use the 63.2% time-constant method
9. Measure the final shunt voltage plateau \(V_final\).

10. Compute the 63.2% level:
```
V_63 = 0.632 × V_final
```

11. Measure the time from the step start to when the trace crosses \(V_63\).
    This time is the RL time constant **τ**.

- Put tape on 60 degrees of the motor
- Mark ~8 spots
- Putting 1V on the motor makes it jump, so clamp to the table to hold position
- Take 8 measurements

12. Compute phase-to-phase inductance:
```
L_pp = τ × R_total
```
use min and max values from the 8 measurements

---

## Measuring Ld and Lq (saliency)

The motor is **12N14P**:
- 14 poles → **7 pole pairs**
- Inductance varies with rotor angle

### Practical method
13. Repeat the step-response measurement while holding the rotor at different positions.
14. Record the minimum and maximum time constants observed.

For the **GBM5208-200T**:
- τ_min ≈ **600 µs**
- τ_max ≈ **1000 µs**

15. Compute phase-to-phase inductances:
```
L_d,pp ≈ τ_min × R_total ≈ 600 µs × 18 Ω ≈ 10.8 mH
L_q,pp ≈ τ_max × R_total ≈ 1000 µs × 18 Ω ≈ 18.0 mH
```

Assign:
- **Ld ≈ minimum inductance**
- **Lq ≈ maximum inductance**

---

## Convert to per-phase inductance (wye connection)

For a symmetric wye motor:
```
L_phase ≈ L_pp / 2
```

So:
```
Ld ≈ 10.8 mH / 2 ≈ 5.4 mH
Lq ≈ 18.0 mH / 2 ≈ 9.0 mH
```

Saliency ratio:
```
Lq / Ld ≈ 1.67
```

---

## Notes for repeatability
- Keep voltage low to avoid rotor motion and heating.
- Ignore initial switching noise; use the exponential envelope.
- Use short leads and, if needed, limit scope bandwidth (~20 MHz).
- Report inductance as a range if rotor angle is uncontrolled.

## Prompt with chatgpt
Walk me through measuring BLDC motor phase resistance and inductance using simple bench equipment.

## Available equipment: ##
- Isolated, adjustable bench DC power supply (voltage & current limit)
- 1 Ω, 10 W resistor
- Earth-grounded Oscilloscope (≥2 channels)
- Multimeter
- BLDC motor (three-phase, disconnected from any controller)

## Connections: ##
PSU+ → Motor lead A → Motor lead B → switch → 1 Ω resistor → PSU−

## Goals: ##
- Measure per-phase resistance (Ω).
- Measure per-phase inductance (H).
- Sanity-check the results against motor physics and expected KV / torque constant.

## Constraints & style: ##
- We will not use an LCR meter
- Use step-response and RL time-constant methods, not impedance analyzers.
- Be explicit about wye vs delta assumptions and conversions.
- Ask me to confirm scope probe placement and safety limits before applying power.
- Keep units consistent and report final inductance in henries (not mH).

## Before beginning  ##
- Ask me for my best guess of the motor’s key specs:
- Approximate KV (rpm/V)
- Expected phase-to-phase resistance range
- Expected inductance range
- Intended application (e.g., balancing robot, gimbal, scooter, etc.)
- Ask what supply voltage and current limit I plan to start with.

## Measurement approach ##
- First: DC resistance measurement using the resistor as a current sense element.
- Second: Inductance measurement using a voltage step, capturing the current rise on the oscilloscope, extracting τ = L/R, and solving for L.
- Explain each step briefly, then wait for my measured values before continuing.

## Behavior: ##
- Do not skip steps.
- Do not assume ideal components.
- Question results that look physically inconsistent.
- Treat this like a real bench experiment, not a textbook problem.

