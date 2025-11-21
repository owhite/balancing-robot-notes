# Preamble

See [this entry](../../DOCS/nov16_IMU.md) on using a Mahony filter to improve readings from the ICM42688. The filter automatically rejects vibration-induced accelerometer noise and uses gyro bias correction to keep the angle accurate over time. The code used for SPI communications with the ICM42688 was slightly modified from [here](https://github.com/finani/ICM42688.git) and the Mahony filter in [supervisor.cpp](src/supervisor.cpp) in `mahonyUpdateIMU()`. Also have a look at `controlLoop()` in [balance_TWR_mode.cpp](src/balance_TWR_mode.cpp) to get a general idea. This work was done at this commit: hash id: `9633b44677406c790000eb56cd1858c216e7f7b5`

Running `./IMU_test.py -p /dev/cu.usbmodem178888901` created the IMU plots of [this entry](../../DOCS/nov16_IMU.md) 

## It's time to test a few things. 

- Verify IMU reports correct angle vs. actual mechanical tilt.
- Quantify the noise the motor injects into the IMU.
- Determine whether torque request → torque output is linear and symmetric.
- Verify identical torque commands produce identical reaction torque.
- Check whether identical torque commands produce repeatable motor currents.
- Timing of torque command applied to motor. Use Teensy pin toggle to mark when torque command is sent. Use oscilloscope to measure phase current rise time, voltage behavior, ESC PWM switching pattern. 

## IMU noise

During the process comparing the IMU with the mechanical angle of the wheel encoders I tested this

I put this the code:
```
    float a_mag = sqrtf(ax*ax + ay*ay + az*az);
    Serial.printf("a_mag=%.5f\r\n", a_mag);
```

and when the motor is running, I get these type of values: 
```
a_mag=3.05135
a_mag=5.25944
a_mag=3.85885
a_mag=6.80106
a_mag=3.69635
a_mag=5.77368
a_mag=9.36287
a_mag=10.89755
a_mag=4.79244
a_mag=2.57672
a_mag=8.32236
a_mag=1.29584
a_mag=2.59398
```

That's pretty horrendous. To reduce noise, I changed the FOC PWM from 20,000 to 10,000, and fooled around with physical dampening of the IMU. and used this to view the vibration: `./IMU_test.py -p /dev/cu.usbmodem178888901` 

[supervisor.cpp](src/supervisor.cpp) applies a Mahony filter which fuses acceleration  and gyro values from the IMU. I added this code: 

```c
  if (accMag > 1e-6f) {
    float recip = 1.0f / accMag;
    ax *= recip;
    ay *= recip;
    az *= recip;

    if (accMag > 0.85f && accMag < 1.15f) {
      accelValid = true;
    } else {
      accelValid = false;
    }
  }
```

Which only blends accel + gyro if vibrations are at a dull roar. This took `a_mag` to < 2, which is reasonable. This is by no means noise free, but it seems reasoanble. ChatG says we could use an improved gating strategy, a hysteresis-based accel valid window, or a frequency-domain vibration rejection trick -- which I have yet to try. 

## Test rigs

This test configuration is not fully representative of real operation—the PCB, IMU, and motors are rigidly mounted to a flat board rather than the full TWR body. Because the robot cannot balance yet, it is difficult to collect realistic motion data while the system is in its operational configuration.

The immediate objective is to characterize and mitigate the vibration coupling before full balancing tests. Planned steps include:
- Implementing on-sensor filtering (MPU6050 DLPF = 20–42 Hz).
- Adding a software low-pass filter and slow gyro-bias estimator in the Teensy firmware.
- Testing mechanical isolation of the IMU (foam or Sorbothane mount).
- Using a tilt-table or tethered setup for controlled experiments prior to free balancing.

**Phase 1: Tilt table approach** 

- Sweep angles slowly by hand (±10°) with motors **off**, then run with motors **on** but wheels off floor.
- Verify roll tracks angle and rate stays quiet with motors energized.
- Try different isolation pads (Sorbothane/silicone) and choose the one that drops high-freq noise most.

**Phase 2 — Safety Tether “Boom Rig”**

Goal: let the controller run upright while preventing catastrophic falls.

Build options (pick one):
- Overhead strap: Hang a light strap from above to a point near the TWR’s CG; length so it catches at ~±8°.
- Side rails: Two vertical 2×4s with furniture sliders; a loose loop of webbing around the body between rails so it can sway but not dump.
- Boom + bearing: Clamp a bicycle wheel (or lazy Susan) horizontally on a stand; bolt a short arm to the hub; tie the TWR top to the arm with a short strap. It can pitch freely but won’t fall far.

Run:
- Start with SAFETY_SCALE = 0.01 and θ̇ gain (K[1]) reduced 20–30%.
- Add software cutout: if |θ| > 12° for >100 ms → disable torque for 1 s and beep.
- Log bursts at 500 Hz for 5–10 s trials. You’ll see immediately if the loop is “grabbing” the upright without chattering.

**Phase 3 — Outriggers (“training wheels”)**
Goal: roll on the ground with a hard angle limit.

- Bolt two small casters on narrow aluminum angles to the chassis sides so they touch the ground at about ±6–8° lean. (Front/back casters work too; the key is limiting pitch.)
- Attempt full balancing on the floor; worst case it gently lands on a wheel, you reset, try again.
- Keep torque clamp low at first (e.g., 1–2 Nm) and ramp up.

**Phase 4 — Structured identification (optional but powerful)**

While tethered or on outriggers:
- Send a low-amplitude chirp torque (0.5→25 Hz over 10 s).
- Record {torque_cmd, roll, roll_rate_filt}.
- Compute the FRF torque→roll_rate in Python (Welch/transfer function). You’ll see the dominant resonance; set your DLPF and (later) a notch accordingly.

