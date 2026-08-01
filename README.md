# GAUG-FINAL-RTS26Summer

#AI DISCLAIMERS
Had ChatGPT touch up the project overview section
Claude was used to port the original single-task blink/HTTP scaffold to the GNSS 1PPS theme, add the interrupt-driven capture path (GPIO ISR, deferred pps_capture_task, and the diagram.json loopback wiring) and add WCET instrumentation and the serial-printed evidence table
Claude assisted with fixing a race condition on the console output with a mutex 

# PROJECT OVERVIEW

This project simulates the 1PPS (one pulse per second) output of a GNSS timing receiver on a dual-core ESP32 and measures the worst-case execution time (WCET) of its tasks. In real GNSS systems, the 1PPS signal is a precise timing pulse that occurs once every second and is used to keep devices like network time servers and communication systems synchronized. Since this signal needs to be extremely accurate, it is handled using a hardware interrupt instead of polling. In this project, Core 1 runs a task that generates the 1PPS signal every second using vTaskDelayUntil, which provides consistent timing without drift. The signal is connected to another GPIO pin configured as an interrupt, causing an ISR to capture a timestamp whenever the pulse occurs. The ISR only performs this quick operation and then wakes another FreeRTOS task, pps_capture_task, which calculates the timing interval and jitter before logging the results. This keeps the interrupt routine fast while moving the longer processing into a normal task. On Core 0, an HTTP server displays the current PPS status and WCET data through a web dashboard, while another low-priority task periodically prints a formatted WCET report to the serial console. Overall, the project demonstrates a realistic interrupt-driven embedded system that includes periodic tasks, hardware interrupts, deferred processing, and a network interface.

# Task Table & WCET Evidence — GNSS 1PPS Monitor

| # | Name               | Core            | Priority                       | Type                    | Trigger / Period                                | Stack                               | Purpose |
| 1 | `pps_task`         | 1 (APP_CPU)     | 5                              | Periodic                | `vTaskDelayUntil`, 1000 ms                      | 2048 words (8 KB)                   | Drives the simulated 1PPS GPIO edge; stands in for the receiver's own PPS-generation hardware. |
| 2 | `pps_isr_handler`  | 0 (ISR context) | interrupt (preempts all tasks) | Hardware ISR            | `GPIO_INTR_POSEDGE` on `PPS_CAPTURE_GPIO`, ~1/s | uses interrupt stack, no task stack | Latches a timestamp the instant the PPS edge arrives; minimal body, no blocking calls, no logging. |
| 3 | `pps_capture_task` | 0 (PRO_CPU)     | 6                              | Deferred / event-driven | `ulTaskNotifyTake` from the ISR, ~1/s           | 3072 words (12 KB)                  | Computes interval/jitter vs. the ideal 1 s spacing and logs it — the "bottom half" of the interrupt. |
| 4 | httpd server task  | 0 (PRO_CPU)     | 5                              | Aperiodic               | Incoming HTTP GET on `/`, `/state`, `/wcet`     | 8192 bytes (`cfg.stack_size`)       | Serves the dashboard page and the JSON state/WCET endpoints. |
| 5 | IDLE0 / IDLE1      | 0 / 1 | 0       | Background                     | Always ready            | FreeRTOS default                                | Housekeeping; not relevant to PPS timing correctness. |

Notes on the design:
- **ISR does the minimum**: it only timestamps, computes one subtraction, and calls `vTaskNotifyGiveFromISR`. Everything else (logging, formatting) is deferred to `pps_capture_task`, so the interrupt's own execution time — which blocks *everything* on that core, including higher-priority tasks — stays as short and predictable as possible.
- **Priority ordering** puts the deferred capture task (6) above the HTTP server (5): a real host prioritizes not losing a timing edge over answering a web request. `pps_task` itself doesn't compete with either — it's pinned to the other core.
- **Core placement**: `gpio_install_isr_service()` is called from `app_main()`, which FreeRTOS/ESP-IDF runs on `PRO_CPU` (Core 0) by default — so the ISR and its deferred task share Core 0 with the HTTP server, while the signal-generation task runs alone on Core 1.

#WCET Evidence Table after running 60 seconds

================== WCET EVIDENCE (uptime 60s) ==================
Task/ISR           Core   Prio     WCET(us)    Samples
--------------------------------------------------------------
pps_task           1      5           14708         60
pps_isr_handler    0      ISR             3         31
pps_capture_task   0      6           12633         32
================================================================

#Final reflection
If I were to continue improving this project, I would add more hardware components to the Wokwi simulation to make it feel more like a real GNSS timing receiver instead of just using the basic GPIO loopback. One thing that surprised me during this project was how difficult it was to intentionally create a reasonable fault. Most of the changes I tried either caused the program to completely break, or they barely affected the system and only removed a small piece of functionality. It was harder than I expected to find a fault that demonstrated a real-time systems issue while still allowing the application to run. The most valuable thing I learned was the connection between GPS timing, the 1PPS signal, and real-time operating systems. Before this project, I did not realize how important hardware interrupts and precise timing are for keeping systems synchronized. I also learned how useful ChatGPT can be for explaining technical concepts and helping me find reliable sources to better understand topics that were new to me.

https://wokwi.com/projects/471101559570443265
