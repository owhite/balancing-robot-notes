# Scheduling Hardware considerations (for Teensy brain board)

FreeRTOS is fine and is put to great use in the MESC code, but the teensy firmware will be structured so the balancing loop is highly deterministic. 

## For a balancing robot
- Clear separation of concerns (control, comms, logging).
- Priority scheduling so background work can’t starve the control path.
- Clean inter-task comms (queues/semaphores) instead of ad-hoc globals.
- Easier to scale as you add features (e.g., gait modes, host RPC).

## Risks & how to avoid them
- Jitter: Don’t rely on a round-robin loop() or a normal RTOS tick to schedule the control law. 
- Use a hardware timer to trigger it (ISR) or a direct-to-task notification that wakes a highest-priority task. 
- Target < ~50–100 µs jitter for a 1 kHz loop.
- Blocking: Never block in the control loop (no printf, no I/O). Push data to a queue/ring buffer and let a lower-priority task handle I/O.
- Interrupt latency: Keep ISRs short; defer work to tasks via xQueueSendFromISR()/vTaskNotifyGiveFromISR().

## Practical layout on Teensy 4.0 (Cortex-M7 @ 600 MHz)
- Timer: Use a PIT/GPT timer at 1 kHz.

## Control loop recommended for RTOS style
- Timer ISR notifies a highest-priority task (no time slicing, no blocking) that executes the control step. 
- Measure jitter; if acceptable, keep it.
- Tasks (descending priority):
- control_task (woken by timer notification; priority highest)
- sensor_fusion_task / IMU (reads/filters IMU; updates a double buffer)
- can_task (parses RX, sends TX; uses queues to/from control)
- logger_task (SD/serial/USB; batch writes to avoid I/O stalls)
- ui/host_task (CLI/telemetry to PC)

## Data movement patterns (safe & fast)
- Double buffers for IMU and setpoints: producer writes buffer A while control reads buffer B; swap with an atomic flag.
- Queues for CAN & logging: xQueueSendFromISR() in ISRs; xQueueReceive() in worker tasks.
- No locks in control: avoid mutexes in the control path.

## Tuning tips
- Set a watchdog and kick it in the control path.
- Give the control task a generous stack (math + FPU).
- Keep the RTOS tick rate modest (e.g., 1 kHz is fine, but don’t depend on it for control timing).
- Measure jitter with a scope (GPIO toggle at loop entry/exit). Adjust priorities/IRQs until it’s within your budget.

