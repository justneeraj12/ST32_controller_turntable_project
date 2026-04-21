#ifndef TURNTABLE_CONTROL_H
#define TURNTABLE_CONTROL_H

#include "main.h"

typedef enum
{
    TURNTABLE_DIR_REVERSE = 0,
    TURNTABLE_DIR_FORWARD = 1
} TurntableDirection_t;

typedef enum
{
    TURNTABLE_STATUS_NOT_INITIALIZED = 0,
    TURNTABLE_STATUS_IN_PROGRESS,
    TURNTABLE_STATUS_REACHED,
    TURNTABLE_STATUS_STALL_FAULT
} TurntableControlStatus_t;

HAL_StatusTypeDef Turntable_ConfigSPI1_Mode3_LongWire(void);
HAL_StatusTypeDef Turntable_ConfigTIM2_Encoder4096(void);
HAL_StatusTypeDef Turntable_InitControl(void);
void Turntable_ClearFault(void);

TurntableControlStatus_t MoveToAngle(float target_angle);

void PowerSTEP01_ApplyStepClock(TurntableDirection_t direction, uint32_t step_clock_hz);
void PowerSTEP01_SoftStop(void);
void PowerSTEP01_HardStop(void);

#endif
