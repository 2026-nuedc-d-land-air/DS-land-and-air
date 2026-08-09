---
name: d-task-uav-router
description: Use when working on the D-task aircraft project, including the air ESP32 bridge, flight-controller firmware, ground station, LoRa V2.3 frames, UART/LX forwarding, ACK behavior, or serial and frozen-log evidence. Read the shared STM core before changing code or judging integration results.
---

# D-Task Aircraft Entry

Read the shared integration core first:
[F:\keil5\stm\skills\SKILL.md](../skills/SKILL.md).

Use the core for common evidence and acceptance discipline, but do not apply its
car-only pin map or motion behavior to aircraft code. Then work in the matching
owner directory:

- `ESP_LX_1`: ESP32 USB diagnostics, frozen log, LoRa parser/scheduler, and LX bridge.
- `ANO_LX`: flight-controller UART adapter, mission state, and FC responses.
- `树莓派地面站`: ground-station protocol, serial receiver, and UI.

For a communication claim, retain the ESP32 serial capture and, when applicable,
its non-destructive `LOG EXPORT` output together with the correlated car/ground
record. Do not accept a UI status alone as proof of an air-side transfer.
