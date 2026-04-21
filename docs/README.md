# Documentation

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
