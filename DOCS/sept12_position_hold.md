## sept 12, 2025
# Attempts at position hold

## Next thing
You want to implement a **motor position-hold test**:  
- Detect RC input stick deflection on channel 0.  
- When deflected beyond ±100 µs from center (1500 µs), capture the current motor position as a target.  
- Run a **PD loop** each tick to hold that position:  
  ```cpp
  torque = Kp * (target_pos - current_pos) + Kd * (0 - current_vel);
  ```  
- Send torque command to ESC via CAN IQREQ message.  

## 🔹 Code Integration Notes
- In `supervisor.cpp`, inside `// Core control loop body`, you will insert the position-hold PD logic.  
- `ESC` already has a `node_id`, but it lives in `esc[i].config.node_id`, not `esc[i].node_id`.  
- Use:  
  ```cpp
  msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID, sup->esc[0].config.node_id);
  ```  
- Be sure to `#include "CAN_helper.h"` in `supervisor.cpp`.  
- Replace `Can1.write(msg)` with `can.write(msg)` (since `controlLoop` already receives `can` as a parameter).  

## 🔹 Position Hold (definition)
“Position hold” = when the robot detects an RC input deflection, it captures the motor’s current encoder position as a reference, and uses a PD controller to apply torque so that the motor resists being moved away from that reference (like an electronic spring-damper lock).  


---

## 🔹 Starter Code Block for Position Hold

Insert this into `supervisor.cpp` where you see `// Core control loop body`:

```cpp
    // ---- Core control loop body ----

    static bool holding = false;
    static float hold_pos_rad = 0.0f;

    // Detect RC stick deflection on channel 0 (raw PWM, ~1500 ± 100 deadband)
    if ((sup->rc_raw[0].raw_us > 1600) || (sup->rc_raw[0].raw_us < 1400)) {
        if (!holding) {
            // First time deflected → capture hold position
            hold_pos_rad = sup->esc[0].state.pos_rad;
            holding = true;
        }
    } else {
        holding = false;
    }

    if (holding) {
        // PD control: torque = Kp*error + Kd*error_dot
        float pos_err = hold_pos_rad - sup->esc[0].state.pos_rad;
        float vel_err = 0.0f - sup->esc[0].state.vel_rad_s;

        const float Kp = 2.0f;   // tune gains
        const float Kd = 0.1f;

        float cmd_torque = Kp * pos_err + Kd * vel_err;

        // Build CAN IQREQ packet
        CAN_message_t msg;
        msg.id = canMakeExtId(CAN_ID_IQREQ,
                              TEENSY_NODE_ID,
                              sup->esc[0].config.node_id);
        msg.len = 8;
        msg.flags.extended = 1;

        canPackFloat(cmd_torque, msg.buf);
        canPackFloat(0.0f, msg.buf + 4);  // second float unused

        // Send to CAN bus via passed-in reference
        can.write(msg);
    }
```

## Bottom line

This works like shit. Nothing to see here. Move on. 
---

### DEETS
**Teensy code:** [here](https://github.com/owhite/MESC_brain_board/tree/main/teensy40/hold_position_test)
