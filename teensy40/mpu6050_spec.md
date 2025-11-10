# MPU6050 Integration Specification for Two-Wheeled Balancing Robot (TWR)

This specification defines the complete, validated approach for using the MPU6050 IMU on a Teensy 4.0 to provide clean, low-noise roll and roll-rate estimates at 1 kHz for a balancing robot.

---

# 1. Electrical Integration

## 1.1 Required Connections

| MPU6050 Pin | Teensy 4.0 Pin | Notes |
|-------------|----------------|-------|
| VCC         | 3V3            | Do **not** use 5V. |
| GND         | GND            | Common ground required. |
| SDA         | SDA (pin 18)   | I²C0 on Teensy. |
| SCL         | SCL (pin 19)   | I²C0 on Teensy. |
| INT         | pin 14         | Any digital pin with interrupts is acceptable. |

## 1.2 IMU Mounting
- Mount IMU **rigidly to the robot body** (not the motor frame).
- Use **gel/foam isolation** only for high-frequency vibration, not for structural motion.
- Z-axis aligned up/down; roll measured around X-axis.

---

# 2. I2C Configuration

## 2.1 Teensy
```cpp
Wire.begin();
Wire.setClock(400000);   // 400 kHz
```

## 2.2 MPU6050 Registers

### Digital Low-Pass Filter (DLPF)
Use bandwidth ~20–21 Hz:
```
CONFIG (0x1A) = 4
```
This removes motor vibration without excessive lag.

### Full-scale ranges
```
GYRO_CONFIG  = 0x10   // ±1000 dps
ACCEL_CONFIG = 0x08   // ±4 g
```

### Sampling
```
SMPLRT_DIV = 0
```
→ sensor outputs at **1 kHz** when DLPF enabled.

### FIFO
Enable accel + gyro:
```
USER_CTRL = 0x40
FIFO_EN   = 0x78
```

---

# 3. INT Pin Usage (Critical)

Using the INT output gives:
- One clean, jitter-free event per new sample (1 kHz).
- Eliminates stale-IMU-read problems.
- Allows you to fetch exactly one fresh sample per control loop.

### Configure INT pin:
```
INT_PIN_CFG (0x37) = 0b00110000   // LATCH_INT_EN + INT_RD_CLEAR
INT_ENABLE  (0x38) = 0b00000001   // DATA_RDY_EN
```

### Teensy ISR
```cpp
volatile bool imu_data_ready = false;

void imu_int_isr() {
    imu_data_ready = true;
}

pinMode(14, INPUT);
attachInterrupt(digitalPinToInterrupt(14), imu_int_isr, RISING);
```

### Clearing the interrupt
After reading FIFO, call:
```cpp
uint8_t status = imu.readReg(0x3A);  // INT_STATUS
```

---

# 4. FIFO Consumption Strategy

For each interrupt event:

1. Read FIFO count.
2. Drain **all but the last frame**.
3. Decode the final accel+gyro frame.
4. Use **only the newest frame** for the Mahony filter.

This ensures:
- No sample backlog
- Every IMU update corresponds to a fresh 1 ms sample

---

# 5. Filtering Requirements

## 5.1 Gyro Bias Estimation (Slow)
Track gyro bias only when vibration is low:

```cpp
if (fabs(gy_dps) < 5 && acc_mag between 0.9 and 1.1 g)
    bias_y += α_bias * (gy - bias_y);
```

Use **α_bias ≈ 0.001** (20-second time constant).

## 5.2 Gyro Low-Pass Filtering (Fast)
For roll-rate used in control:

Corner freq: **20 Hz**
```
roll_rate_filt += α_lpf * (gyro_corr - roll_rate_filt)
```

α_lpf for 1 kHz sampling:
```
α_lpf = 1 - exp(-2π * 20 / 1000) ≈ 0.11
```

## 5.3 Complementary / Mahony Filter
Use modified Mahony with **accelerometer trust factor**:

```
trust = 1 / (1 + 20 * |acc_mag - 1.0|)
```

- trust ≈ 1 → low vibration
- trust ≪ 1 → motor noise, accelerometer unreliable

Apply trust to both:
- proportional correction
- integral drift correction

---

# 6. IMU Outputs for the Balancing Controller

All angles/rates must be given in **radians** and **rad/s**.

- **roll_rad** = imu.roll * DEG_TO_RAD
- **roll_rate_rad_s** = imu.roll_rate * DEG_TO_RAD

Write to supervisor:

```cpp
sup->imu.roll        = roll_rad;
sup->imu.roll_rate   = roll_rate_rad_s;
sup->imu.pitch       = pitch_rad;       // not used in TWR
sup->imu.yaw         = 0;
sup->imu.valid       = true;
```

---

# 7. Control Loop Timing

- Teensy control loop runs at **1 kHz** via IntervalTimer ISR.
- Each tick:
  - Check `imu_data_ready`
  - If true:
    - Clear flag
    - Call `imu.update()`

### DO NOT run imu.update() unconditionally.  
This causes:
- stale samples
- random lag
- huge RMS roll-rate

---

# 8. Telemetry & Debug

Send only:
```json
{"t": ..., "roll": <rad>, "roll_rate": <rad/s>}
```

Plot in Python:
- Roll (deg or rad)
- Roll rate
- RMS (normalized over ~200 samples)
- Motor on/off tests

Normal RMS(roll_rate):
- Motors off: < 0.5–1.5 deg/s
- Motors on: 2–5 deg/s
- When falling: spikes are normal

If RMS stays >20 deg/s → IMU update rate mismatch or stale samples.

---

# 9. Failure Modes & Diagnostics

| Symptom | Cause |
|--------|-------|
| Roll stays constant | imu.update() not getting fresh FIFO data (INT missing) |
| Roll jumps wildly | accel trust too high; vibration corrupts gravity vector |
| Roll_rate huge noise | gyro bias not compensated or LPF not applied |
| IMU update slow | FIFO not drained, or I2C errors |
| Balancer oscillates | wrong units (deg vs rad), unfiltered state, or bad signs |

---

# 10. Summary: What You Must Implement

✅ **INT pin wired & ISR to flag new samples**  
✅ **Interrupt-driven FIFO read** (not timer-based guessing)  
✅ **Gyro bias tracking**  
✅ **Gyro low-pass filtering (20 Hz)**  
✅ **Mahony with variable accelerometer trust**  
✅ **Export roll & roll_rate in radians**  
✅ **Use filtered roll_rate for LQR**  
✅ **Plot & verify RMS stability**  

Once all above is implemented, the IMU subsystem is *sufficient for stable balancing*.

---

# Appendices
- Appendix A: Register reference for INT and DLPF
- Appendix B: Gyro/accel scaling constants
- Appendix C: Recommended test procedures

