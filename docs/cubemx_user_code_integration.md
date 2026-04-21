# CubeMX USER CODE Integration

Use these snippets inside Cube-generated USER CODE blocks so your logic survives regeneration.

## 1) `Core/Src/main.c` include section

Insert in `/* USER CODE BEGIN Includes */`:

```c
#include "turntable_control.h"
```

## 2) `Core/Src/main.c` init section

Insert in `/* USER CODE BEGIN 2 */` after `MX_SPI1_Init();` and `MX_TIM2_Init();`:

```c
if (Turntable_InitControl() != HAL_OK)
{
  Error_Handler();
}
```

## 3) `Core/Src/main.c` loop section

Insert in `/* USER CODE BEGIN WHILE */`:

```c
TurntableControlStatus_t tt_status;

/* Request any angle; controller indexes to nearest of 0, 90, 180 deg. */
tt_status = MoveToAngle(90.0f);

if (tt_status == TURNTABLE_STATUS_STALL_FAULT)
{
  /* Hard stop already triggered internally, handle alarm/log here. */
}
```

## 4) SPI1 requirement (Mode 3)

`Turntable_InitControl()` applies:

- `CLKPolarity = SPI_POLARITY_HIGH`
- `CLKPhase = SPI_PHASE_2EDGE`
- `BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256`

## 5) TIM2 encoder requirement (4096 PPR)

`Turntable_InitControl()` applies:

- Encoder mode: `TIM_ENCODERMODE_TI12`
- Counter period: `4095`
- Runtime angle conversion: `count * 360 / 4096`
