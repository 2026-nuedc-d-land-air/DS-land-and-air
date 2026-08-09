---
name: d-task-car-entry
description: Use when working in the D-task STM32 car repository. Route every firmware, wiring, protocol, log-analysis, flashing, or acceptance request to the shared STM D-task core before making changes.
---

# D-Task Car Entry

The authoritative local skill is the shared core:
[F:\keil5\stm\skills\SKILL.md](../../skills/SKILL.md).

Read it before changing car source, pin ownership, line-following behavior,
LoRa V2.3 handling, calibration, speed gates, flashing, or test evidence. Load
only the core references required by the request. Keep field logs as evidence;
do not replace them with inferred UI state or unverified documentation.
