# MESC Motor Control Variables
A listing of variables in the [MESC firmware](https://github.com/davidmolony/MESC_Firmware) that are particularly relevant to balancing. 

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
  - All normalized requests (`UART_req`, `remote_ADC`) are scaled by these bounds.

- **`mtr[0]->FOC.enc_angle`**  

  - A 16-bit fixed-point mechanical angle, derived from the encoder timer’s CNT, scaled so that it runs 0–65536 per cycle, and resynchronized by the encoder’s Z-pulse via CCR1. 
  - `mtr[0]->FOC.enc_angle` is calculated as:
  - enc_angle=pole_pairs×enc_ratio×(CNT−CCR3)+offset
  - `CNT` gives the live position, while `CCR3` (index pulse latch) ensures the angle resets to a known reference once per mechanical revolution.
  - `CNT` → current quadrature count (mechanical position since last Z).
     - → This provides the absolute reference point per revolution.
  - Relative count
      - (`CNT` - `CCR3`) → ticks since last Z-index pulse.
  - `enc_ratio` converts raw encoder ticks → fixed-point [0…65536] mechanical rev.
  - Then multiplied by `pole_pairs` → converts to magnetic motor position.
  - If `encoder_polarity_invert` is set, it flips the direction by subtracting from 65536.
  - `enc_offset` allows calibration of zero-angle alignment (phase alignment between encoder mechanical zero and motor electrical zero).

- **`mtr[0]->FOC.abs_position`**

    Created for this project. Notice this code addition:
    ```c
	#ifdef POSVEL_PLANE
    if(_motor->FOC.encoder_polarity_invert){
        // Invert direction for abs_position as well
        _motor->FOC.abs_position = ((uint16_t)(_motor->enctimer->Instance->CCR3) - (uint16_t)(_motor->enctimer->Instance->CNT)) & 0x0FFF; // 12-bit wrap
    } else {
        _motor->FOC.abs_position = ((uint16_t)(_motor->enctimer->Instance->CNT)  - (uint16_t)(_motor->enctimer->Instance->CCR3)) & 0x0FFF; // 12-bit wrap
    }
    #endif
    ```
  - The raw mechanical encoder position (0–4096) adjusted to an absolute reference using the Z-pulse (`CCR3`)
  - `FOC.abs_position → ( (uint16_t)(CNT) - (uint16_t)(CCR3) ) & 0x0FFF;` // for 12-bit encoder
  - `CNT` → current encoder count (0–4095).
  - `CCR3` → latched value of CNT when the Z-pulse occurred.
  - `(CNT - CCR3)` → ticks since the last index (Z-pulse).
  - Note: if it wasnt obvious already 4095 is hardcoded all over the place in MESC.

---

# Current MESC Settings

| Parameter             | Value             |
| --------------------- | ----------------- |
| adc1                  | 640               |
| adc1_max              | 2700              |
| adc1_min              | 500               |
| adc1_pol              | 1.000000          |
| adc2_max              | 4095              |
| adc2_min              | 1200              |
| adc2_pol              | 1.000000          |
| can_adc               | 3                 |
| ehz                   | -0.283872         |
| error                 | 0                 |
| error_all             | 0                 |
| FOC_Advance           | 0.000000          |
| FOC_angle             | 40500             |
| FOC_curr_BW           | 3000.000000       |
| FOC_enc_ang           | 40500             |
| FOC_enc_oset          | 40500             |
| FOC_enc_pol           | 1                 |
| FOC_enc_PPR           | 4096              |
| FOC_flux_gain         | 0.367423          |
| FOC_flux_gain         | 0.367423          |
| FOC_flux_nlin         | 5000.000000       |
| FOC_fpwm              | 20000.000000      |
| FOC_fw_ehz            | 0.000000          |
| FOC_hall_array_ok     | 0                 |
| FOC_hall_iir          | 0.950000          |
| FOC_hall_Vt           | 2.000000          |
| FOC_hfi_eHz           | 0.000000          |
| FOC_hfi_type          | 1                 |
| FOC_hfi_volt          | 1.000000          |
| FOC_Max_Mod           | 0.950000          |
| FOC_obs_type          | 1                 |
| FOC_ol_step           | 10                |
| FOC_ortega_gain       | 1000000.000000    |
| Hall_flux             | Array[12]         |
| id                    | -0.007379         |
| input_opt             | 32                |
| iq                    | -0.000819         |
| iqreq                 | 0.000000          |
| meas_cl_curr          | 8.500000          |
| meas_curr             | 20.000000         |
| meas_volt             | 4.000000          |
| node_id               | 12                |
| opt_app_type          | 0                 |
| opt_circ_lim          | 2                 |
| opt_cont_type         | 0                 |
| opt_fw                | 2                 |
| opt_hall_start        | false             |
| opt_lr_obs            | false             |
| opt_motor_temp        | false             |
| opt_mtpa              | 0                 |
| opt_phase_bal         | false             |
| opt_pwm_type          | 0                 |
| par_dir               | 0                 |
| par_flux              | 0.001350          |
| par_fw_curr           | 0.000000          |
| par_i_max             | 50.000000         |
| par_i_min             | -50.000000        |
| par_i_park            | 0.000000          |
| par_ibat_max          | 10.000000         |
| par_ld                | 0.000015          |
| par_lq                | 0.000022          |
| par_motor_sensor      | 4                 |
| par_p_max             | 3000.000000       |
| par_pp                | 14                |
| par_r                 | 0.070000          |
| par_rpm_max           | 0                 |
| par_SL_sensor         | 0                 |
| password              |                   |
| safe_count            | 100               |
| safe_start            | 100               |
| speed_ki              | 0.100000          |
| speed_kp              | 0.500000          |
| speed_req             | 0.000000          |
| TMOS                  | 299.611572        |
| TMOT                  | 199.968307        |
| uart_dreq             | 0.000000          |
| uart_req              | 0.000000          |
| vbus                  | 29.723145         |
| Vd                    | -0.004284         |
| Vq                    | 0.006748          |
