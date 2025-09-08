# MESC Brain Board

## Approach: a separation of concerns
* Teensy: Runs the high-level brain → gait generator, balance estimator, RC inputs
* MESC: Handles the low-level muscle control → torque or velocity loops per joint
* ESP32: Easy to log and analyze later → pumps UDP to the computer
* Desktop computer → perfectly adequate way of viewing high speed data with UDP

## Project plan:
Gonna try to use my open source motor controller, the [MP2-DFN](https://github.com/owhite/MP2-DFN), and this circuit for a [brain board](https://github.com/owhite/MESC_brain_board/blob/main/brainboardV1.0/MESC_brain_board.pdf), a teensy 4.0, and an ESP-32, in combination with [MESC firmware](https://github.com/davidmolony/MESC_Firmware).

## Many thanks
This project would never happen without:
* [MESC firmware](https://github.com/davidmolony/MESC_Firmware). 
* [Netzpfuscher's](https://github.com/Netzpfuscher/TTerm) incredible TTerm and CN work
* [MP2](https://github.com/badgineer/MP2-ESC), an open source motor controller from badgineer. 

## Teensy 4.0
* 600+ MHz ARM Cortex-M7 — plenty of headroom for filtering, control loops, gait logic, telemetry
* Tons of I/O for IMU, encoders, RC input, WiFi (via ESP add-on), etc.
* Precise timing with elapsedMicros or interrupts for deterministic control
* Great for real-time control, sensor fusion, and behavior modeling

## MESC motor controller
* Executes low-level motor control (Field-Oriented Control)
* DIY and commercially available boards
* Handles current, velocity, or position control
* Built-in support for encoders, FOC, and torque estimation
* Accurate low-level motor control using current sensing
* Reads angular position from the MT6701 encoder via SPI or PWM
* UART/CAN interface (RX + TX)	Real-time communication with the Teensy
* PWM / FOC commutation w/ efficient, smooth motor operation

## Brain board features
* Teensy 4.0
* ESP32
* RC receiver connector (for PWM/PPM/SBUS input)
* ESP32 UART serial programmer
* Power input connector
* Voltage divider (to monitor battery voltage)
* Push-button E-stop connector
* Power button (for system on/off)
* IMU connector (e.g., MPU-6050)
* Buzzer (for system alerts or E-stop signal)
* MESC I/O
* Reserved 2 GPIO pins on Teensy for emergency shutoff to MESC
* CAN connector (for MESC motor control)

## Specs and testing
* Using the MT6701 [[LINK](DOCS/MT6701.md)]
* Implementing CAN [[LINK](DOCS/CAN.md)]
* Measuring jitter on the MESC [[LINK](DOCS/jitter_testing.md)]
* **PRELIMINARY:** Control loop [[LINK](DOCS/control_loop.md)]
* **PRELIMINARY:** Brain board firmware [[LINK](DOCS/software_specs.md)]
* Balancing checklist [[LINK](DOCS/balancing_checklist.md)]

### CAN Bus
* Full-duplex communication between Teensy and motor controllers
* Carries:
  * Commands: set_torque, set_velocity, enable_motor, zero_encoder, etc.
  * Telemetry: encoder position, estimated torque, velocity, fault codes
* Real-time performance with minimal latency
* Can support multiple motor controllers on the same bus

### MT6701 Encoder
* High-resolution magnetic rotary encoder (up to 14-bit)
* Measures rotor position for the BLDC
* Output via SPI, PWM, or ABI 
* Provides absolute angle, ideal for FOC and position tracking

## NOT IMPLEMENTED
* Optional I²C header (for sensors like INA219 or IMU)
* Encoder inputs (if reading position directly)
* SWD debug access for Teensy
* Big red e-stop button
* Voltage divider (to monitor battery voltage)

## STILL NEEDS WORK
* Not great design w/ DC-DC conversion from high voltage VBat
* replace out the IMU for a ICM-42688-P

## VERSIONS
* See github tag [pcb-v1](https://github.com/owhite/MESC_brain_board/releases/tag/pcb-v1) for V1.0
* Retreive with this command: "git checkout pcb-v1"

## NOTES
Ever wanted to change all the names of kicad files at once?

use this:
```
python3 replace_kicad_names.py VESC_brain_board MESC_brain_board
```

[[LINK](replace_kicad_names.py)]
