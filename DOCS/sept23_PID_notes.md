## Sept 23, 2025
# PID Control Issues and Context for Balancing Robot

At this stage: 
- PD seems to supply reasonable control law on the motor.
- Can send data from the control plant to a python graphing program
- The graphing program is able to send rudimentary commands to teensy 

## 1. Motor Does Not Move With Authority
- In the current PD setup, the motor reaches setpoint, but it does so with **low torque output**.
- It “just does enough” to get there, which works for position control but **feels weak**.
- For a balancing robot (an unstable plant), this lack of authority is a serious problem — the controller must react strongly and decisively to disturbances.

---

## 2. Boat Steering Analogy
- Imagine steering a boat against a strong current.
- With light grip, the wheel drifts — that’s like a weak PD controller.
- With a **tight grip**, you resist disturbances with authority and hold course — that’s the kind of strong control needed for balancing.

---

## 3. Why Adding the I-Term Matters
- PD alone always “backs off” near the setpoint, which explains why torque never ramps up enough to firmly hold position.
- An **I-term integrates error over time**, applying steady torque to cancel biases (friction, load imbalance).
- This provides the “leaning into the wheel” effect — resisting drift and ensuring exact steady-state alignment.
- Anti-windup is needed to keep the integral from running away if torque saturates.

---

## 4. Limits of Hand-Tuning PID
- Raising Kp quickly leads to oscillations.
- Increasing Kd adds damping but can amplify noise and cause vibrations.
- Balancing requires very high loop speed, careful filtering, and often a tedious, frustrating manual tuning process.

---

## 5. Connection to LQR
- LQR (Linear Quadratic Regulator) provides a **systematic alternative**:
  - Define what errors are costly (tilt angle, wheel position).
  - Define how costly actuator effort is.
  - The solver finds optimal gains.
- The effects we want — **stiffness (P)**, **damping (D)**, **steady-state correction (I)**, and **authority** — can all be achieved within LQR.
- Instead of endless knob-turning, you adjust **cost weights** and let the math do the tuning.

---

## 6. State-Space Model Outline (Inverted Pendulum on Wheels)

We can describe the balancing robot using a **state vector**:

\[
x = \begin{bmatrix}
\theta \\   % tilt angle
\dot{\theta} \\   % angular velocity
x_w \\   % wheel position
\dot{x}_w \\   % wheel velocity
\end{bmatrix}
\]

Where:
- \(\theta\): tilt angle of the body (0 = upright).
- \(\dot{\theta}\): angular velocity of the tilt.
- \(x_w\): wheel displacement.
- \(\dot{x}_w\): wheel velocity.

The linearized dynamics can be written as:

\[
\dot{x} = A x + B u
\]

Where:
- \(u\): motor torque / force command.
- \(A, B\): system matrices derived from pendulum-on-cart physics.

### LQR Control Law
The LQR controller computes:

\[
u = -K x
\]

Where \(K\) is obtained by solving the **Riccati equation** given cost matrices:

- \(Q\): weights on state error (make tilt angle costly).
- \(R\): weights on control effort (torque/current cost).

By choosing large weight on tilt angle in \(Q\), the controller behaves with **strong authority**, just like a tightly held boat wheel.

---

## Next improvements
- Current PD lacks authority.
- Balancing will require strong corrective torque (like tightly holding a boat wheel in a current).
- Adding an I-term can improve steady-state authority.
- Ultimately, LQR should let us achieve the same effects without manual trial-and-error tuning.
