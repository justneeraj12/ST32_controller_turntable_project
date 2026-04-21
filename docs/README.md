# Documentation

## Architecture Migration Summary

- Legacy setup: Arduino + DM542 open-loop step command path (no encoder feedback in control loop).
- Current setup: STM32 HAL closed-loop control on NUCLEO-G0B1RE + X-NUCLEO-IHM03A1 using encoder feedback from TIM2.
- Control implementation source: [firmware/App/turntable_control.c](firmware/App/turntable_control.c).
- Integration guide for USER CODE blocks: [docs/cubemx_user_code_integration.md](docs/cubemx_user_code_integration.md).

## Source-of-Truth Note (main.c and .ioc)

- In this workspace snapshot, no CubeMX project main.c or .ioc file is present.
- Therefore, this document reflects the active control logic from [firmware/App/turntable_control.c](firmware/App/turntable_control.c) and the integration contract in [docs/cubemx_user_code_integration.md](docs/cubemx_user_code_integration.md).
- Once main.c and .ioc are committed, this section should be updated with final peripheral pin names and initialization ordering.

## SPI Validation Test

- Date: 2026-04-21
- Board Stack: NUCLEO-G0B1RE + X-NUCLEO-IHM03A1
- Result: PASS
- Observation: STATUS register transactions over SPI1 are valid and the firmware prints IHM03A1 Driver Found.

## SPI1 Configuration Used for PowerSTEP01

The PowerSTEP01 on X-NUCLEO-IHM03A1 requires SPI Mode 3.

- CPOL: High
- CPHA: 2-Edge
- Data Size: 8-bit
- NSS: Software-controlled
- Prescaler: 256 (low clock for long-wire stability)

The generated STM32 HAL implementation is in [firmware/App/turntable_control.c](firmware/App/turntable_control.c).

## Encoder Feedback GPIO (Servo Calibrator to TIM2)

Configured mapping for encoder feedback:

- Channel A: PA0 (TIM2_CH1, Arduino A0)
- Channel B: PA1 (TIM2_CH2, Arduino A1)
- Reference Ground: GND to GND
- Encoder Supply (if required by the sensor): 3.3V

If CubeMX pin assignment is changed later, update this section to match the generated IOC configuration.

## Device Profile

PID constants currently used by the turntable controller:

- $K_p = 26.0$
- $K_i = 2.4$
- $K_d = 0.18$

Constants are defined in [firmware/App/turntable_control.c](firmware/App/turntable_control.c).

## Turntable and Assembly Piston Synchronization Logic

Current synchronization model is state-based using MoveToAngle status returns:

1. Command indexing with MoveToAngle(target_angle).
2. While status is TURNTABLE_STATUS_IN_PROGRESS, the piston action remains blocked.
3. When status is TURNTABLE_STATUS_REACHED, the turntable is at index (0, 90, or 180 degrees) and piston actuation is allowed.
4. If TURNTABLE_STATUS_STALL_FAULT is reported, controller triggers HARD_STOP and piston actuation must remain inhibited until fault clear.

The fault path is implemented by command-versus-motion supervision in [firmware/App/turntable_control.c](firmware/App/turntable_control.c), where low encoder movement under sustained command triggers PowerSTEP01_HardStop.
