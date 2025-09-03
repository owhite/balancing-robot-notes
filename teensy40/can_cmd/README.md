## Teensy CAN Test Programs

**Teensy sketches** that can be loaded to send commands over CAN to the MESC-based ESC. Each one uses a different control channel built into the firmware.  

---

### 1. `can_id_adc1_2_req.cpp`
- Use **`can_id_adc1_2_req.cpp`** for simple normalized throttle testing.  
- **CAN ID:** `CAN_ID_ADC1_2_REQ (0x010)`  
- **Payload:**  
  - Bytes 0–3 = `throttle_mapped` (float, normalized between **-1.0 and +1.0**)  
  - Bytes 4–7 = unused (`0.0f`)  
- **ESC Behavior:**  
  - Treated like a remote throttle input.  
  - Internally assigned to `motor_curr->input_vars.remote_ADC1_req`.  
  - Negative values = reverse torque, positive = forward torque.  
- **How to use:**  
  - Open Serial Monitor at 115200 baud.  
  - Type a float between `-1.0` and `+1.0`.  
  - Teensy sends that throttle value over CAN once.  
  - The ESC will respond if its **`remote_ADC_can_id`** matches the Teensy’s node ID (set via terminal: `set can_adc <id>`).  

---

### 2. `can_id_iqreq.cpp`
- Use **`can_id_iqreq.cpp`** for direct current/torque control.  
- **CAN ID:** `CAN_ID_IQREQ (0x001)`  
- **Payload:**  
  - Bytes 0–3 = `Iq_req` (float, in **amperes of q-axis current**)  
  - Bytes 4–7 = unused (`0.0f`)  
- **ESC Behavior:**  
  - Written directly into `motor_curr->FOC.Idq_req.q`.  
  - This is raw torque control, bypassing throttle mapping.  
- **How to use:**  
  - Open Serial Monitor at 115200 baud.  
  - Type a float (e.g. `5.0` or `-3.0`).  
  - Teensy will continuously stream that torque request at 500 Hz.  
  - The ESC will respond if its **`remote_Iq_can_id`** matches the Teensy’s node ID (set via terminal: `set can_iq <id>`).  

---

### Debugging in STM32CubeIDE
If you want to trace the command as it flows through the ESC firmware, good debug breakpoints are:  

- **`CAN1_RX0_IRQHandler()`** → Confirms the packet arrived in the ISR and was queued.  
- **`TASK_CAN_packet_cb()`** → Confirms the packet was dispatched to the right handler (ADC1_2_REQ or IQREQ).  
- **`MESCinput_Collect()`** → Confirms the chosen input value (CAN vs UART vs ADC) is being forwarded into the motor control loop.  

