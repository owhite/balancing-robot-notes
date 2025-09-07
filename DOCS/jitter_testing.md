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

**4.1 Enable the DWT cycle counter (once at startup)**

```c
// Call during early init (before starting control loops)
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // enable DWT/ITM
DWT->CYCCNT = 0;                                // reset counter
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // start cycle counter
```

**4.2 Measure timing at the top of fastLoop()**

Adjust EXPECTED_HZ to your actual FOC/fastLoop rate (e.g., 20 kHz).
Uses SystemCoreClock for portability (168 MHz on STM32F405 by default).

```c
// ---- at file scope in MESCfoc.c ----
#include <limits.h>

#ifndef EXPECTED_HZ
#define EXPECTED_HZ 20000u // fastLoop() expected call rate (Hz)
#endif

static uint32_t fl_last_cycles = 0;
static int32_t  fl_jitter_min  = INT32_MAX;  // cycles error
static int32_t  fl_jitter_max  = INT32_MIN;  // cycles error
static int64_t  fl_jitter_sum  = 0;          // sum of cycles error
static uint32_t fl_jitter_cnt  = 0;

static inline float cycles_to_us(float cycles) {
  return cycles / ((float)SystemCoreClock / 1e6f);
}

// ---- at the very beginning of fastLoop() ----
{
  const int32_t expected_cycles = (int32_t)(SystemCoreClock / EXPECTED_HZ);

  uint32_t now  = DWT->CYCCNT;
  uint32_t diff = now - fl_last_cycles;  // handles wrap-around safely
  fl_last_cycles = now;

  // error vs expected in *cycles*
  int32_t err_cyc = (int32_t)diff - expected_cycles;

  if (err_cyc < fl_jitter_min) fl_jitter_min = err_cyc;
  if (err_cyc > fl_jitter_max) fl_jitter_max = err_cyc;
  fl_jitter_sum += err_cyc;
  fl_jitter_cnt++;

  // Every ~100 ms at EXPECTED_HZ (e.g. 2000 samples @ 20 kHz)
  if (fl_jitter_cnt >= (EXPECTED_HZ / 10u)) {
    const float avg_cyc = (float)fl_jitter_sum / (float)fl_jitter_cnt;

    // Convert to microseconds for reporting
    const int16_t min_us_i16 = (int16_t)cycles_to_us((float)fl_jitter_min);
    const int16_t max_us_i16 = (int16_t)cycles_to_us((float)fl_jitter_max);
    const float   avg_us     = cycles_to_us(avg_cyc);

    CAN_send_jitter(min_us_i16, max_us_i16, avg_us);

    // reset window
    fl_jitter_min = INT32_MAX;
    fl_jitter_max = INT32_MIN;
    fl_jitter_sum = 0;
    fl_jitter_cnt = 0;
  }
}
```
**4.3 Define a telemetry CAN ID**
```c
// can_ids.h
#define CAN_ID_JITTER  0x350  // pick any unused base ID in your scheme
```

**4.4 Send a compact 8-byte CAN frame (10 Hz)**
- Payload layout (8 bytes total):
- bytes 0–1: int16_t min_jitter_us
- bytes 2–3: int16_t max_jitter_us
- bytes 4–7: float avg_jitter_us

Use whichever CAN helper you already have (HAL or your TASK_CAN_* wrapper). Below are two examples; use one that matches your codebase.

**(A) Using HAL directly (adjust can1.hw, IDs as needed):**

```c
#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef *can1_hw; // e.g., can1.hw in your code
extern uint8_t ESC_NODE_ID;        // sender
extern uint8_t BRAIN_NODE_ID;      // receiver (or 0 for broadcast)

static inline uint32_t make_ext_id(uint16_t msg_id, uint8_t receiver, uint8_t sender) {
  return ((uint32_t)msg_id << 16) | ((uint32_t)receiver << 8) | sender;
}

void CAN_send_jitter(int16_t min_us, int16_t max_us, float avg_us) {
  uint8_t data[8];
  memcpy(&data[0], &min_us, 2);
  memcpy(&data[2], &max_us, 2);
  memcpy(&data[4], &avg_us, 4);

  CAN_TxHeaderTypeDef tx = {0};
  tx.IDE   = CAN_ID_EXT;
  tx.RTR   = CAN_RTR_DATA;
  tx.DLC   = 8;
  tx.ExtId = make_ext_id(CAN_ID_JITTER, BRAIN_NODE_ID, ESC_NODE_ID);

  uint32_t mailbox;
  HAL_CAN_AddTxMessage(can1_hw, &tx, data, &mailbox);
}
```
**(B) Using your existing MESC CAN task helper:**
```c
void CAN_send_jitter(int16_t min_us, int16_t max_us, float avg_us) {
  uint8_t payload[8];
  memcpy(&payload[0], &min_us, 2);
  memcpy(&payload[2], &max_us, 2);
  memcpy(&payload[4], &avg_us, 4);

  // Example wrapper you may have:
  // TASK_CAN_tx_ext(CAN_ID_JITTER, /*receiver*/BRAIN_NODE_ID, /*sender*/can1.node_id, payload, 8);
}
```
**4.5 What the Teensy/PC sees**
At 10 Hz, you’ll receive a compact summary of fastLoop() timing variation in microseconds:

- min_jitter_us: most negative deviation in the last 0.1 s
- max_jitter_us: most positive deviation in the last 0.1 s
- avg_jitter_us: average deviation over the window

You can print or plot these alongside your other telemetry to confirm that added bus traffic or firmware changes do not degrade control-loop determinism.

**Notes & Tips**
- If your fastLoop() frequency isn’t 20 kHz, just set EXPECTED_HZ accordingly.
- DWT->CYCCNT wraps every ~25.5 s at 168 MHz; because we take differences each call, wrap is safe.
- For even more context you can add a sequence counter to this frame so the Teensy can detect any lost jitter packets.
