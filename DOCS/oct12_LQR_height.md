## Oct 12, 2025
# Pendulum Model Height

## Pendulum Control Bandwidth and Implications for Balancing Robot

During testing of the inverted pendulum system, the LQR controller produced stable but sluggish responses, even when control effort (R value) was reduced or motor torque increased. The limiting factor was identified as the relationship between the pendulum’s physical dynamics and the controller’s update rate.

For the test configuration:
- Length (pivot to COM): ≈ 0.13 m
- Mass: ≈ 0.19 kg
- Moment of inertia: ≈ 4.9 × 10⁻³ kg·m²
- Natural frequency: ≈ 5.9 rad/s (≈ 1.1 s period)
- Control loop: 500 Hz (2 ms sampling period)

This configuration resulted in a slow plant (long period, weak torque leverage) controlled by a moderately fast digital loop. Because torque authority and gravitational restoring forces were small, the controller could not strongly influence the state between samples—making the response appear “sluggish” despite increased LQR gains.

When the pendulum length is shortened (≈ 0.07 m), the natural frequency increases (~20 rad/s), and the control authority improves substantially. A shorter pendulum is easier to stabilize digitally because its dynamics operate well within the controller’s effective bandwidth.

This outcome is opposite to human intuition: a tall broomstick is easier for a person to balance because human reaction time is slow, whereas a digital controller benefits from a faster, more responsive plant.

## According to ChatG...
For the two-wheel balancing robot, this limitation is far less severe:

- The robot’s actuation (wheel acceleration) directly moves the base under the COM, giving much greater control authority.
- The 500 Hz loop frequency is adequate for body dynamics in the 15–50 rad/s range, typical for small robots.
- Stability will instead depend on motor torque limits, mechanical delay, and COM height—not on sample rate alone.

In summary:

The hanging pendulum revealed that long, slow plants are difficult to dominate with discrete-time control.
For the balancing robot, faster actuation and lower COM ensure the same 500 Hz controller will perform effectively.
