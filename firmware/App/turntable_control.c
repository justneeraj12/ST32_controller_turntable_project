#include "main.h"

#include <math.h>
#include <stdint.h>

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

typedef enum
{
    POWERSTEP_DIR_REV = 0,
    POWERSTEP_DIR_FWD = 1
} PowerSTEP_Direction_t;

/* Replace these weak hooks with your project driver integration. */
__weak void PowerSTEP01_ApplyStepClock(PowerSTEP_Direction_t direction, uint32_t step_clock_hz)
{
    (void)direction;
    (void)step_clock_hz;
}

__weak void PowerSTEP01_Stop(void)
{
}

#define ENCODER_COUNTS_PER_REV      4096.0f
#define POSITION_TOLERANCE_DEG      0.5f
#define PID_KP                       28.0f
#define PID_KI                       2.8f
#define PID_KD                       0.22f
#define PID_INTEGRAL_LIMIT           250.0f
#define STEPCLK_MIN_HZ               30.0f
#define STEPCLK_MAX_HZ               3500.0f
#define SAFETY_SLOWDOWN_WINDOW_DEG   22.0f
#define SAFETY_MIN_SCALE             0.25f

static float Clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float NormalizeDegrees360(float angle_deg)
{
    while (angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float WrapSignedDegrees(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float ReadEncoderDegrees_TIM2(void)
{
    uint32_t count = __HAL_TIM_GET_COUNTER(&htim2) % (uint32_t)ENCODER_COUNTS_PER_REV;
    return ((float)count * 360.0f) / ENCODER_COUNTS_PER_REV;
}

static float SafetyScaleNearFobRiskAngles(float current_deg)
{
    static const float risk_angles_deg[2] = {90.0f, 180.0f};
    float min_distance = 360.0f;
    uint32_t i;

    for (i = 0U; i < 2U; ++i)
    {
        float distance = fabsf(WrapSignedDegrees(risk_angles_deg[i] - current_deg));
        if (distance < min_distance)
        {
            min_distance = distance;
        }
    }

    if (min_distance >= SAFETY_SLOWDOWN_WINDOW_DEG)
    {
        return 1.0f;
    }

    return SAFETY_MIN_SCALE +
           (1.0f - SAFETY_MIN_SCALE) * (min_distance / SAFETY_SLOWDOWN_WINDOW_DEG);
}

void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 * Closed-loop position command for TIM2 encoder mode (4096 PPR).
 * Call this function periodically (for example every 5-10 ms).
 * Returns signed StepClock command in Hz for diagnostics.
 */
float MoveToPosition(float target_degrees)
{
    static float integral_term = 0.0f;
    static float previous_error = 0.0f;
    static uint32_t last_tick_ms = 0U;

    uint32_t now_tick_ms = HAL_GetTick();
    float dt_sec;
    float current_deg;
    float error_deg;
    float derivative_term;
    float pid_output;
    float step_clock_hz;
    float safety_scale;
    PowerSTEP_Direction_t direction;

    target_degrees = NormalizeDegrees360(target_degrees);

    if (last_tick_ms == 0U)
    {
        last_tick_ms = now_tick_ms;
        return 0.0f;
    }

    dt_sec = ((float)(now_tick_ms - last_tick_ms)) / 1000.0f;
    last_tick_ms = now_tick_ms;
    if (dt_sec <= 0.0f)
    {
        dt_sec = 0.001f;
    }

    current_deg = ReadEncoderDegrees_TIM2();
    error_deg = WrapSignedDegrees(target_degrees - current_deg);

    if (fabsf(error_deg) <= POSITION_TOLERANCE_DEG)
    {
        integral_term = 0.0f;
        previous_error = error_deg;
        PowerSTEP01_Stop();
        return 0.0f;
    }

    integral_term += error_deg * dt_sec;
    integral_term = Clampf(integral_term, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

    derivative_term = (error_deg - previous_error) / dt_sec;
    previous_error = error_deg;

    pid_output = (PID_KP * error_deg) + (PID_KI * integral_term) + (PID_KD * derivative_term);

    safety_scale = SafetyScaleNearFobRiskAngles(current_deg);
    step_clock_hz = fabsf(pid_output) * safety_scale;
    step_clock_hz = Clampf(step_clock_hz, STEPCLK_MIN_HZ, STEPCLK_MAX_HZ);

    direction = (pid_output >= 0.0f) ? POWERSTEP_DIR_FWD : POWERSTEP_DIR_REV;
    PowerSTEP01_ApplyStepClock(direction, (uint32_t)step_clock_hz);

    return (direction == POWERSTEP_DIR_FWD) ? step_clock_hz : -step_clock_hz;
}
