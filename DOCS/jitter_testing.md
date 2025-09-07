## Measuring fastLoop() Timing Jitter

### 1. Generating a Test Signal on the STM32F405 (MESC)

I added these functions to MESC code:

```c
// PB5 setup
static inline void jitter_pin_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; (void)RCC->AHB1ENR;

  GPIOB->MODER   = (GPIOB->MODER & ~(3U << (5U*2))) | (1U << (5U*2));
  GPIOB->OTYPER &= ~(1U << 5);
  GPIOB->OSPEEDR |= (3U << (5U*2));
  GPIOB->PUPDR &= ~(3U << (5U*2));
  GPIOB->BSRR = (1U << 5) << 16;
}

static inline void jit_set(void)    { GPIOB->BSRR =  (1U << 5); }
static inline void jit_clr(void)    { GPIOB->BSRR = ((1U << 5) << 16); }
static inline void jit_toggle(void) {  GPIOB->ODR ^= (1U << 5); }

```

Then with MESC_Firmware/MESC_Common/Src/MESCfoc.c

add this into the end of **fastLoop()**

```c
static inline void jit_toggle(void) {  GPIOB->ODR ^= (1U << 5); }
```

Put a scope on PB5 and get this:

<img src="scope1.png" alt="Block diagram" width="400"/>

### Result:
- **Each toggle = one call to `fastLoop()`**, so:
  - Square wave frequency = half the loop frequency.
- Example: observing ~10 kHz on oscilloscope → means `fastLoop()` executes at ~20 kHz.

---

### 2. Measure the output with the Teensy 4.0
- Connected ESC test pin → **Teensy pin 9** (interrupt-capable).
- On the brain board this is the PWM in header for reading an RC transmitter
- Teensy runs an **ISR on rising edges**:
  - Timestamps each edge with the **ARM cycle counter** (`ARM_DWT_CYCCNT`).
  - Stores timestamps into a **ring buffer** (ISR-safe).
- Main loop:
  - Computes **period (µs)** = time difference between edges.
  - Computes **jitter/error (µs)** = deviation from target period.
  - Outputs NDJSON over USB serial, e.g.:
    ```json
    {"seq":1234,"t_us":1234567.890,"period_us":50.0,"target_us":50.0,"err_us":0.05}
    ```

---

### 3. Host PC Logging & Plotting
- Python script reads Teensy’s serial NDJSON stream.
- Parses JSON lines and stores into rolling buffers.
- Plots in real-time using Matplotlib:
  - **Period vs. sample**
  - **Jitter vs. sample**
  - **Jitter histogram** (distribution shape)
  - **Jitter violin plot** (spread + density)

Run script:
```
python3 ./plot_teensy_jitter.py /dev/cu.usbmodem178888901
```

Typical result showing there is a very small amount of fastLoop() delays:

<img src="plot1.png" alt="Plot result" width="600"/>

This example shows virtually no change after increasing CAN messages to 500kz:

<img src="plot2.png" alt="Plot result" width="600"/>


### Outcome
- Jitter measured in **sub-µs range**, with only rare small spikes.
- Demonstrates loop timing is stable enough for deterministic FOC.
- Workflow (ESC pin toggle → Teensy capture → Python plots) provides a **portable jitter analyzer** for real-time firmware validation.
- This will be useful to test if other changes to MESC code have a negative impact
- CAN running at 500hz doesnt seemt to impact anything

---
### 4. Internal Jitter Measurement on the STM32F405 (DWT + 10 Hz CAN Telemetry)

In addition to the external pin toggle + Teensy capture, we will also measure jitter directly inside the STM32F405 using the Cortex-M4 DWT cycle counter. We’ll aggregate min/max/average error over ~100 ms and export a small CAN telemetry frame at 10 Hz so the Teensy/PC can monitor loop health.

**4.1 Create a new struct in MESCfoc.h** 

```c
#ifdef POSVEL_PLANE
// Jitter metrics for fastLoop() entry-to-entry period, measured via DWT->CYCCNT
typedef struct {
    int32_t  min_cyc, max_cyc;
    int64_t  sum_cyc;
    uint32_t samples;
    uint32_t last_entry_cyc;
    int32_t  expected_cyc;
    float    avg_us; // average jitter over all samples, in microseconds
    float    p2p_us; // peak-to-peak jitter over all samples, in microseconds
    volatile uint8_t clear_req;
} MESCjitter_s;
#endif
```

and add this to the big motor typedef:

```c
typedef struct{
	TIM_HandleTypeDef *mtimer; //3 phase PWM timer
.
.
.
	input_vars_t input_vars;
	MESClrobs_s lrobs;
	MESCoptionFlags_s options;
	bool conf_is_valid;
#ifdef POSVEL_PLANE
    MESCjitter_s jitter;     // fastLoop() timing jitter metrics
#endif
}MESC_motor_typedef;
```

**4.2 Measure timing at the top of fastLoop()**

```c
void fastLoop(MESC_motor_typedef *_motor) {
	uint32_t cycles = CPU_CYCLES;

#ifdef JITTER_TEST
	//
	// god forgive the idiot that puts something at the top of fastLoop()
	//
	uint32_t period = cycles - _motor->jitter.last_entry_cyc;   // wrap-safe
	_motor->jitter.last_entry_cyc = cycles;

	if (period != 0u) { // skip first sample
	    int32_t jitter_cyc = (int32_t)period - _motor->jitter.expected_cyc;

	    if (jitter_cyc < _motor->jitter.min_cyc) _motor->jitter.min_cyc = jitter_cyc;
	    if (jitter_cyc > _motor->jitter.max_cyc) _motor->jitter.max_cyc = jitter_cyc;
	    _motor->jitter.sum_cyc += jitter_cyc;
	    _motor->jitter.samples++;
	}

	// Resets after request from the task
	if (_motor->jitter.clear_req) {
	    _motor->jitter.min_cyc = INT32_MAX;
	    _motor->jitter.max_cyc = INT32_MIN;
	    _motor->jitter.sum_cyc = 0;
	    _motor->jitter.samples = 0;
	    _motor->jitter.clear_req = 0;
	}

#endif
```
**4.3 Define a telemetry CAN ID**
```c
// can_ids.h
#define CAN_ID_JITTER  0x350  // pick any unused base ID in your scheme
```

**4.4 Send a compact 8-byte CAN frame using the current helper**
- Payload layout (8 bytes total):
- bytes 0–3: float jitter.avg_us
- bytes 4–7: float jitter.p2p_us

```c
void TASK_CAN_telemetry_fast(TASK_CAN_handle * handle){

	MESC_motor_typedef * motor_curr = &mtr[0];

	TASK_CAN_add_float(handle	, CAN_ID_ADC1_2_REQ	  	, CAN_BROADCAST, motor_curr->input_vars.ADC1_req		, motor_curr->input_vars.ADC2_req	, 0);
	TASK_CAN_add_float(handle	, CAN_ID_SPEED		  	, CAN_BROADCAST, motor_curr->FOC.eHz		, 0.0f					, 0);
	TASK_CAN_add_float(handle	, CAN_ID_BUS_VOLT_CURR 	, CAN_BROADCAST, motor_curr->Conv.Vbus		, motor_curr->FOC.Ibus	, 0);
	TASK_CAN_add_uint32(handle	, CAN_ID_STATUS	  		, CAN_BROADCAST, motor_curr->MotorState		, 0						, 0);
	TASK_CAN_add_float(handle	, CAN_ID_MOTOR_CURRENT 	, CAN_BROADCAST, motor_curr->FOC.Idq.q		, motor_curr->FOC.Idq.d	, 0);
	TASK_CAN_add_float(handle	, CAN_ID_MOTOR_VOLTAGE 	, CAN_BROADCAST, motor_curr->FOC.Vdq.q		, motor_curr->FOC.Vdq.d	, 0);

#ifdef POSVEL_PLANE
	// these values are somewhat untested in that I've
	//  never been able to make fastLoop() slow down
	uint32_t n       = motor_curr->jitter.samples;
    int32_t  min_cyc = motor_curr->jitter.min_cyc;
    int32_t  max_cyc = motor_curr->jitter.max_cyc;
    int64_t  sum_cyc = motor_curr->jitter.sum_cyc;

    if (n > 0 && min_cyc != INT32_MAX && max_cyc != INT32_MIN) {
        const float cyc2us = 1.0f / ((float)SystemCoreClock / 1e6f);
        motor_curr->jitter.avg_us = ((float)sum_cyc / (float)n) * cyc2us;
        motor_curr->jitter.p2p_us = (float)(max_cyc - min_cyc) * cyc2us;

        TASK_CAN_add_float(handle, CAN_ID_JITTER, CAN_BROADCAST, motor_curr->jitter.avg_us, motor_curr->jitter.p2p_us, 0);

        motor_curr->jitter.clear_req = 1; // ask ISR to reset window
    }
#endif

}

Print or plot these alongside other telemetry to confirm that added bus traffic or firmware changes do not degrade control-loop determinism.

**Notes & Tips**
- If your fastLoop() frequency isn’t 20 kHz, just set EXPECTED_HZ accordingly.
- DWT->CYCCNT wraps every ~25.5 s at 168 MHz; because we take differences each call, wrap is safe.
- For even more context you can add a sequence counter to this frame so the Teensy can detect any lost jitter packets.
