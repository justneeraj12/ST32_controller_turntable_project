#ifndef POWERSTEP01_HAL_H
#define POWERSTEP01_HAL_H

#include "turntable_control.h"

HAL_StatusTypeDef PowerSTEP01_Init_24V_Agilent(void);
HAL_StatusTypeDef PowerSTEP01_TestClosedLoop90deg(void);

/* Optional integration hook: implement with a timer/PWM pulse source driving STCK. */
void PowerSTEP01_SetStepClockFrequency(uint32_t step_clock_hz);

#endif
