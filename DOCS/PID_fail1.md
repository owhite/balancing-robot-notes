# Examples of PID not working at this point. 

Using this code block, and many different settings for Kp and Kd, I got no where with trying to get motor to move to position under PID control. 

```c
void run_mode_set_position(Supervisor_typedef *sup,
                           FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &can) {
    static bool first_entry = true;
    static unsigned long start_time = 0;

    const float Kp = 0.04f;
    const float Kd = 0.005f;
    const float setpoint = M_PI;   // target position

    if (first_entry) {
        logIndex = 0;
        first_entry = false;
        start_time = micros();
    }

    // --- PID control ---
    float pos_err = setpoint - sup->esc[0].state.pos_rad;
    if (pos_err >  M_PI) pos_err -= 2.0f * M_PI;
    if (pos_err < -M_PI) pos_err += 2.0f * M_PI;

    float vel_err = 0.0f - sup->esc[0].state.vel_rad_s;

    float p_term = Kp * pos_err;
    float d_term = Kd * vel_err;
    float cmd_torque_raw = p_term + d_term;

    // clamp torque
    const float TORQUE_CLAMP = 0.8f;
    float cmd_torque = constrain(cmd_torque_raw, -TORQUE_CLAMP, TORQUE_CLAMP);

    // --- Output command over CAN ---
    CAN_message_t msg;
    msg.id = canMakeExtId(CAN_ID_IQREQ, TEENSY_NODE_ID,
                          sup->esc[0].config.node_id);
    msg.len = 8;
    msg.flags.extended = 1;
    canPackFloat(cmd_torque, msg.buf);
    canPackFloat(0.0f, msg.buf + 4);
    can.write(msg);

    // --- Logging ---
    if (logIndex < 1000) {
        logBuffer[logIndex++] = {
            micros() - start_time,
            setpoint,
            sup->esc[0].state.pos_rad,
            sup->esc[0].state.vel_rad_s,
            pos_err,
            cmd_torque,
	    p_term,
	    d_term
        };
    } else {
        // Done collecting: print JSON burst
        Serial.println("{ \"samples\":[");
        for (int i = 0; i < 1000; i++) {
	  Serial.printf(
			"{\"t\":%lu,\"setpoint\":%.4f,\"pos\":%.4f,"
			"\"vel\":%.4f,\"err\":%.4f,"
			"\"torque\":%.4f,\"p_term\":%.4f,\"d_term\":%.4f}%s\r\n",
			logBuffer[i].t_us,
			logBuffer[i].setpoint,
			logBuffer[i].pos,
			logBuffer[i].vel,
			logBuffer[i].error,
			logBuffer[i].torque,
			logBuffer[i].p_term,
			logBuffer[i].d_term,
			(i < 999) ? "," : ""
			);

        }
        Serial.println("]}\r\n");

        // Reset and exit to idle
        sup->mode = SUP_MODE_IDLE;
        first_entry = true;
    }
}

```

Resulting plots are here. This shows a good example: 

<img src="IMAGES/PID_fail1.1.png" alt="Plot result" width="600"/>

And these are sucktastic
<img src="IMAGES/PID_fail1.2.png" alt="Plot result" width="600"/>

<img src="IMAGES/PID_fail1.3.png" alt="Plot result" width="600"/>

## General conclusions

- there is very little reproducibility
- the same settings result different results
- at the risk going down rabbit holes you can consider all the issues [here](torque_nonlinearity.md)
