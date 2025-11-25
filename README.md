# MESC BRAIN BOARD 

A PCB and software development project for a balancing robot. 

The goal is to use a hobby Electronic Speed Controller (one open source [example](https://github.com/owhite/MP2-DFN), and this circuit for a [brain board](https://github.com/owhite/MESC_brain_board/blob/main/brainboardV1.0/MESC_brain_board.pdf), a teensy 4.0, and an ESP-32, in combination with [MESC firmware](https://github.com/davidmolony/MESC_Firmware).

## GENERAL APPROACH

A separation of concerns is divided across these architectures:

* **Teensy:** Runs the high-level brain → gait generator, balance estimator, RC inputs
* **MESC:** The firmware on the ESC for control → torque or velocity loops per joint
* **ESP32:** Easy to log and analyze later → pumps UDP to the computer
* **Desktop computer** → perfectly adequate way of viewing high speed data with UDP

## TEENSY 4.0
* 600+ MHz ARM Cortex-M7 — plenty of headroom for filtering, control loops, gait logic, telemetry
* Tons of I/O for IMU, encoders, RC input, WiFi (via ESP add-on), etc.
* Precise timing with elapsedMicros or interrupts for deterministic control
* Great for real-time control, sensor fusion, and behavior modeling

## MESC MOTOR CONTROLLER
* Deploys on simple DIY or commercially available ESC boards
* Executes low-level Field-Oriented Control (FOC) motor control 
* Built-in support for encoders, FOC, and torque estimation
* Accurate low-level motor control using current sensing
* Reads angular position from the MT6701 encoder via SPI or PWM
* UART/CAN interface (RX + TX)	Real-time communication with the Teensy
* PWM / FOC commutation w/ efficient, smooth motor operation

## OTHER BRAIN BOARD FEATURES
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

## CAN BUS
* Full-duplex communication between Teensy and motor controllers
* Carries:
  * Commands: set_torque, set_velocity, enable_motor, zero_encoder, etc.
  * Telemetry: encoder position, estimated torque, velocity, fault codes
* Real-time performance with minimal latency
* Can support multiple motor controllers on the same bus

## SPECS AND TESTING
* Using the MT6701 [[LINK](DOCS/MT6701.md)] → works great but required some minor modifications to MESC position and [velocity measurement](DOCS/pos_vel_CAN.md)
* Measuring jitter on the MESC [[LINK](DOCS/jitter_testing.md)] → none found at current operating speeds
* Determinism discussion [[LINK](DOCS/determinism_design.md)]
* Implementing CAN [[LINK](DOCS/CAN.md)]
* Preliminary balancing checklist [[LINK](DOCS/balancing_checklist.md)]
* Balance failure modes [[LINK](DOCS/balancing_failures.md)]
* Notes about MESC [[LINK](DOCS/MESC_control_variables.md)]
* Discovering the inadequacies of [PID](DOCS/PID_position.md). 
* Notes about learning [LQR](DOCS/learn_LQR.md). 
* How the brain board firmware works: [teensy code](DOCS/teensy_firmware_walkthrough.md). 
* **BLATHER AND NOISE:** Brain board firmware [[LINK](DOCS/software_specs.md)]

## Laboratory notes, 2025

* [Position hold](DOCS/sept12_position_hold.md)
* [Controller issues](DOCS/sept18_controller_issues.md)
* [Torque nonlinearity](DOCS/sept19_torque_nonlinearity.md)
* [PID notes](DOCS/sept23_PID_notes.md)
* [Least squares](DOCS/sept30_least_squares.md)
* [Torque response](DOCS/oct03_torque_response.md)
* [LQR modeling](DOCS/oct11_LQR_modeling.md)
* [LQR height](DOCS/oct12_LQR_height.md)
* [IMU testing](DOCS/nov16_IMU.md)

## MANY THANKS
This project would never happen without:
* [MESC firmware](https://github.com/davidmolony/MESC_Firmware). 
* [Netzpfuscher's](https://github.com/Netzpfuscher/TTerm) incredible TTerm and CAN work.
* [MP2](https://github.com/badgineer/MP2-ESC) an open source motor controller from badgineer. 

## Let's be real

You might assume building a balancing robot is mostly a matter of assembling parts and flashing someone else’s code onto it. I used to think that too — until I built one. What I’ve learned is that balancing robots aren’t really built, they’re tuned. Everything about them depends on tight interactions between hardware, sensors, motors, and feedback loops, and those interactions are different for every single robot.

A balancing robot is a dynamic control system, not a static device. It’s constantly trying to predict its own motion, measure its own tilt, compensate for delays, cancel vibration, and stabilize itself against gravity — hundreds of times per second. That means even small differences in hardware completely change the way the controller behaves. For example:

- Motors & ESCs: Different torque curves, different dead zones, different current limits, different FOC tuning.
- Wheels & tires: Diameter, traction, inertia, compliance — all change how the robot responds to torque.
- IMU: Sensitivity, noise, mounting vibration, sample timing, bias drift — all affect measured tilt.
- Microcontroller timing: Loop rate, jitter, scheduling delays, filter performance — all shift the controller behavior.
- Physical build: Height of the center of mass, frame stiffness, mass distribution — all change the robot’s dynamics.

If you change any of these, even slightly, the controller behaves differently. That’s why tuning is so time-consuming: you’re adjusting a control law to match the physics and imperfections of your specific robot.

You are welcome to take my code and run it on your bot. But remember:
- You can absolutely start with my code
- It will not balance your robot without a lot of tuning.
- What's most important to use this project as a prompt for making your own system
- tuning is the real work — not a flaw, but part of the fun
- What really makes a robot stand is eliminating all the problems like jitter, timing issues, CAN communications, and modeling

Just like musical instruments or high-performance cars, balancing robots need to be customized, adjusted, and dialed in until the system “feels right.” The code is just the beginning; the tuning is the craft.

## The really great news is there is a new source of help. 

I want to be transparent about this: building this robot wasn’t just me writing code. I've written code for 20 plus years, but in this case I worked through the entire project with ChatGPT as a kind of engineering partner. ChatG helps with asking questions, testing ideas, refining approaches, debugging weird behavior, and iterating the design over and over.

A balancing robot is a complex closed-loop control problem. As far as I know I never would have been able to use Google to address how robots behaves. ChatGPT creates the ability to explore those problems conversationally:

- When I wasn’t sure whether the IMU filtering was right, I asked it to analyze my data.
- It is incredible at helping with plotting results
- When I had motor vibration I couldn’t explain, I used chat to brainstorm possible causes and experiments.
- It taught me everything about LQR, interpretted equations, supplied python code to perform most of the calculations. It involved a lot of debugging -- when the LQR gains seemed wrong, I walked through the math with it.
- When the ESC behavior looked inconsistent, we debugged line by line until we understood it.
- When I needed to test an assumption, it helped me design the test.
- When something behaved strangely, I could describe it and get three possible hypotheses to try.

The project was far less like copying code from the internet and more like having a very patient senior engineer building one-off code bundles to eventually get this thing to stand. 

**We live in a world of prompt-driven engineering:** I shaped my thinking through prompts, refined those prompts based on the robot’s behavior, and used those conversations to build understanding that would have taken weeks on my own. ChatGPT didn’t “solve the problem” — it helped me reason through it, step by step.

So if you see this robot standing and balancing in a video, know that behind the scenes was not just hardware and code — it was hundreds of micro-conversations, experiments, adjustments, and iterations. ChatGPT was the scaffolding that helped me structure that process.

## NOTES
Ever wanted to change all the names of kicad files at once?

use this:
```
python3 replace_kicad_names.py VESC_brain_board MESC_brain_board
```

[[LINK](replace_kicad_names.py)]

## STILL NEEDS WORK
* Not great design w/ DC-DC conversion from high voltage VBat
* Replace out the IMU for a ICM-42688-P
* Order / revise pushbuttons for programming the ESP32
* Ensure 3.3V CAN transciever

## VERSIONS
* See github tag [pcb-v1](https://github.com/owhite/MESC_brain_board/releases/tag/pcb-v1) for V1.0
* Retreive with this command: "git checkout pcb-v1"

