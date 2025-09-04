## MESC Motor Control Variables

### 🔹 Input & Control Variables

- **`UART_req`**  
  - Main user input variable.  
  - Written from **UART commands** or **CAN handlers** (`CAN_ID_ADC1_2_REQ`, `CAN_ID_IQREQ`).  
  - Passed into `MESCinput_Collect()` and merged into `FOC.Idq_req.q`.  
  - Typically treated as normalized `[-1.0 … +1.0]`, then scaled by `max_request_Idq.q` / `min_request_Idq.q`.

- **`remote_ADC1_req` / `remote_ADC2_req`**  
  - Populated from **`CAN_ID_ADC1_2_REQ`** packets.  
  - Mirror the function of physical ADC throttle inputs but come via CAN.  
  - Flow into the same path as throttle inputs.

- **`remote_Iq_req`**  
  - Intended variable for remote torque/current commands via CAN (`CAN_ID_IQREQ`).  
  - In your firmware branch, this was bypassed and written directly to `UART_req` instead.  
  - Author’s style suggests `remote_Iq_req` is the canonical variable.

- **`remote_ADC_timeout` / `remote_Iq_timeout`**  
  - Countdown timers reset whenever a new CAN packet arrives.  
  - If they expire (no packets for a few cycles), input is invalidated → ESC zeros the torque command for safety.

- **`FOC.Idq_req.q`**  
  - Final **q-axis current request** (in amps).  
  - This is what the inner current control loop actually tries to track.  
  - Computed inside the control loop from inputs (`UART_req`, speed loops, etc.).  
  - Should **not** be written to directly from CAN/UART — it gets overwritten each cycle.

- **`max_request_Idq.q` / `min_request_Idq.q`**  
  - Configured current limits (positive/negative).  
  - Define the maximum torque the ESC will ever command.  
  - All normalized requests (`UART_req`, remote_ADC) are scaled by these bounds.

---

