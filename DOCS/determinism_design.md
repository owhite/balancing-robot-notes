# Determinism in the Brain Board Firmware

This document summarizes the design choices that keep the **balancing control loop deterministic** on the Teensy 4.0 brain board. The goal is to ensure that the 500–1000 Hz control-plane runs with minimal jitter (<50–100 µs) and is not disrupted by background I/O or logging.

---

## 1. Control Loop Triggering

- **Hardware timer (IntervalTimer)**
  - The control loop is triggered by a dedicated hardware timer (`g_ctrlTimer`).
  - The ISR does **not** perform heavy math; it only sets a flag (`g_control_due`).
  - The actual control computation runs in `loop()` when the flag is set.

**Impact:**  
Short ISR + hardware timer → consistent tick timing with minimal jitter.

---

## 2. Priority Ordering in `loop()`

The `loop()` function is structured in strict priority order:

1. **High priority:** if `g_control_due`, run `controlLoop()` immediately.  
2. **Medium priority:** poll CAN bus, enqueue RX frames, parse with `handleCANMessage()`.  
3. **Low priority:** update LEDs, speaker, pushbutton state, telemetry.  

**Impact:**  
Control always executes first. Lower-priority tasks never starve or delay the control path.

---

## 3. CAN Message Handling

- **Polling into a ring buffer**:  
  ```cpp
  while (Can1.read(msg)) {
      canBufferPush(canRxBuf, msg);
  }
  while (canBufferPop(canRxBuf, msg)) {
      handleCANMessage(msg);
  }
  ```
- The RX ISR is lightweight; heavy parsing is deferred to the main loop.
- CAN messages update ESC state variables (`pos_rad`, `vel_rad_s`, temps, etc.) with minimal memcpy operations.

**Impact:**  
No blocking or heavy logic inside ISRs → avoids introducing jitter.

---

## 4. Telemetry (Serial1 → ESP32)

- **TelemetryPacket (~56 B)** is prepared each cycle with `loadTelemetryPacket()`.
- Direct calls to `sendTelemetryPacket()` are **throttled** to a lower frequency (20–50 Hz), not sent every control tick.
- UART is initialized with a high baud rate (≥500 kbaud) to ensure packets drain fast.
- Optionally, use `availableForWrite()` to drop packets instead of blocking when buffer is full.

**Impact:**  
Telemetry runs at background rate, never blocks the 1 kHz control loop.

---

## 5. Non-blocking Background Tasks

- **LEDs** and **speaker** are updated with non-blocking state machines (`led_update()`, `tone_update()`).
- **Pushbutton** is debounced in software with microsecond timers, not delay loops.
- No `delay()`, `delayMicroseconds()`, or blocking waits inside `loop()`.

**Impact:**  
Background chores consume negligible time, ensuring deterministic control scheduling.

---

## 6. Avoiding Blocking I/O

- **No `Serial.printf` or `Serial.println` in the control loop.**
- Debug logging (e.g., CAN packet dumps) is only used during development and not in the real-time path.
- Telemetry and logging are designed as **background tasks**, not part of the critical loop.

**Impact:**  
Eliminates the risk of serial back-pressure introducing jitter.

---

## 7. Shared State and ESC Data

- ESC state variables(e.g., `pos_rad`, `vel_rad_s`, `MOS_temp`) are updated in `handleCANMessage()`.
- Reads happen in the control loop.  
- Although not double-buffered yet, updates are small single-word memcpy operations, reducing race condition risk.
- Double-buffering can be added later if needed for multi-variable consistency.

**Impact:**  
Lightweight state updates keep CAN parsing deterministic without blocking.

---

## 8. Measurement and Validation

- Jitter testing performed with scope toggling and DWT cycle counter confirms stable timing at ~20 kHz FOC loop and 500–1000 Hz control loop.
- CAN bus load (~25–50%) did not measurably affect loop timing.
- Telemetry is intentionally designed not to run at control frequency.

**Impact:**  
Empirical validation backs up the architectural choices.

---

## Key Principles Followed

- **ISR does the minimum necessary** (set a flag, push a byte).  
- **Control loop runs first** in priority order.  
- **Background I/O is decoupled** and throttled.  
- **No blocking calls in the control path.**  
- **Telemetry and logging are lossy by design** (okay to drop, never to block).  

---

### Conclusion

By structuring the firmware around these principles preserves a highly deterministic control path for the balancing robot. The control-plane runs at 500–1000 Hz with minimal jitter, while background tasks like CAN parsing, LED/speaker updates, and telemetry operate asynchronously without interfering with determinism. Measurement and testing will ensure that minimal jitter enters into the system. 
