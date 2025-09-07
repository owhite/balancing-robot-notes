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

- **`mtr[0]->FOC.enc_angle`**  

A 16-bit fixed-point electrical angle, derived from the encoder timer’s CNT, scaled so that it runs 0–65536 per electrical cycle, and resynchronized by the encoder’s Z-pulse via CCR1. 
`mtr[0]->FOC.enc_angle` is calculated as:

enc_angle=pole_pairs×enc_ratio×(CNT−CCR3)+offset

`CNT` gives the live position, while `CCR3` (index pulse latch) ensures the angle resets to a known reference once per mechanical revolution.

`CNT` → current quadrature count (mechanical position since last Z).
`CCR3` → captured counter value at the last Z-pulse (index pulse).
→ This provides the absolute reference point per revolution.

Relative count
(`CNT` - `CCR3`) = ticks since last Z-index pulse.

`enc_ratio` converts raw encoder ticks → fixed-point [0…65536] mechanical rev.

Then multiplied by `pole_pairs` → converts to electrical cycles.

If `encoder_polarity_invert` is set, it flips the direction by subtracting from 65536.

`enc_offset` allows calibration of zero-angle alignment (phase alignment between encoder mechanical zero and motor electrical zero).

- **`mtr[0]->FOC.abs_position`**  

  - the raw mechanical encoder position (0–4096) adjusted to an absolute reference using the Z-pulse (CCR3)
  - FOC.abs_position = ( (uint16_t)(CNT) - (uint16_t)(CCR3) ) & 0x0FFF; // for 12-bit encoder
  - CNT = current encoder count (0–4095 if it’s a 12-bit / 4096 CPR encoder).
  - CCR3 = latched value of CNT when the Z-pulse occurred.
  - (CNT - CCR3) = ticks since the last index (Z-pulse).

Notice this code addition:
```c
#ifdef POSVEL_PLANE
	_motor->FOC.abs_position = ( (uint16_t)(_motor->enctimer->Instance->CNT) - (uint16_t)(_motor->enctimer->Instance->CCR3) ) & 0x0FFF; // for 12-bit encoder
#endif
```
---

