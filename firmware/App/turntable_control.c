#include "turntable_control.h"

#include <math.h>
#include <stdbool.h>

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

#define ENCODER_PPR                        4096.0f
#define ENCODER_PERIOD_COUNTS              4095U
#define POSITION_TOLERANCE_DEG             0.6f
#define PID_KP                             26.0f
#define PID_KI                             2.4f
#define PID_KD                             0.18f
#define PID_INTEGRAL_LIMIT                 300.0f
#define STEPCLK_MIN_HZ                     40.0f
#define STEPCLK_MAX_HZ                     3600.0f
#define SAFETY_ZONE_DEG                    20.0f
#define SAFETY_ZONE_MIN_SCALE              0.25f
#define STALL_MIN_COMMAND_HZ               180.0f
#define STALL_MIN_MOVEMENT_DEG             0.4f
#define STALL_TIMEOUT_MS                   350U

typedef struct
{
    float integral;
    float prev_error;
    float last_angle_deg;
    uint32_t last_tick_ms;
    uint32_t last_motion_tick_ms;
    bool initialized;
    bool fault_latched;
} TurntableControllerState_t;

static TurntableControllerState_t g_ctrl = {0};
static const float k_index_angles_deg[3] = {0.0f, 90.0f, 180.0f};

__weak void PowerSTEP01_ApplyStepClock(TurntableDirection_t direction, uint32_t step_clock_hz)
{
    (void)direction;
    (void)step_clock_hz;
}

__weak void PowerSTEP01_SoftStop(void)
{
}

__weak void PowerSTEP01_HardStop(void)
{
}

static float clampf(float value, float min_value, float max_value)
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

static float normalize_deg(float angle_deg)
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

static float wrap_signed_deg(float angle_deg)
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

static float read_encoder_deg(void)
{
    uint32_t count = __HAL_TIM_GET_COUNTER(&htim2) & ENCODER_PERIOD_COUNTS;
    return ((float)count * 360.0f) / ENCODER_PPR;
}

static float nearest_index_angle(float target_angle)
{
    float target = normalize_deg(target_angle);
    float best_angle = k_index_angles_deg[0];
    float best_distance = fabsf(wrap_signed_deg(target - k_index_angles_deg[0]));
    uint32_t i;

    for (i = 1U; i < 3U; ++i)
    {
        float d = fabsf(wrap_signed_deg(target - k_index_angles_deg[i]));
        if (d < best_distance)
        {
            best_distance = d;
            best_angle = k_index_angles_deg[i];
        }
    }

    return best_angle;
}

static float slowdown_scale(float current_angle_deg)
{
    float d90 = fabsf(wrap_signed_deg(90.0f - current_angle_deg));
    float d180 = fabsf(wrap_signed_deg(180.0f - current_angle_deg));
    float d = (d90 < d180) ? d90 : d180;

    if (d >= SAFETY_ZONE_DEG)
    {
        return 1.0f;
    }

    return SAFETY_ZONE_MIN_SCALE +
           (1.0f - SAFETY_ZONE_MIN_SCALE) * (d / SAFETY_ZONE_DEG);
}

static TurntableControlStatus_t check_stall_and_guard(float command_hz,
                                                       float current_angle_deg,
                                                       uint32_t now_ms)
{
    if (fabsf(command_hz) < STALL_MIN_COMMAND_HZ)
    {
        g_ctrl.last_angle_deg = current_angle_deg;
        g_ctrl.last_motion_tick_ms = now_ms;
        return TURNTABLE_STATUS_IN_PROGRESS;
    }

    if (fabsf(wrap_signed_deg(current_angle_deg - g_ctrl.last_angle_deg)) >= STALL_MIN_MOVEMENT_DEG)
    {
        g_ctrl.last_angle_deg = current_angle_deg;
        g_ctrl.last_motion_tick_ms = now_ms;
        return TURNTABLE_STATUS_IN_PROGRESS;
    }

    if ((now_ms - g_ctrl.last_motion_tick_ms) >= STALL_TIMEOUT_MS)
    {
        g_ctrl.fault_latched = true;
        PowerSTEP01_HardStop();
        return TURNTABLE_STATUS_STALL_FAULT;
    }

    return TURNTABLE_STATUS_IN_PROGRESS;
}

HAL_StatusTypeDef Turntable_ConfigSPI1_Mode3_LongWire(void)
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

    return HAL_SPI_Init(&hspi1);
}

HAL_StatusTypeDef Turntable_ConfigTIM2_Encoder4096(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = ENCODER_PERIOD_COUNTS;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 6;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 6;

    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
    {
        return HAL_ERROR;
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
    {
        return HAL_ERROR;
    }

    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    return HAL_OK;
}

HAL_StatusTypeDef Turntable_InitControl(void)
{
    HAL_StatusTypeDef status;

    status = Turntable_ConfigSPI1_Mode3_LongWire();
    if (status != HAL_OK)
    {
        return status;
    }

    status = Turntable_ConfigTIM2_Encoder4096();
    if (status != HAL_OK)
    {
        return status;
    }

    g_ctrl.integral = 0.0f;
    g_ctrl.prev_error = 0.0f;
    g_ctrl.last_angle_deg = read_encoder_deg();
    g_ctrl.last_tick_ms = HAL_GetTick();
    g_ctrl.last_motion_tick_ms = g_ctrl.last_tick_ms;
    g_ctrl.initialized = true;
    g_ctrl.fault_latched = false;

    return HAL_OK;
}

void Turntable_ClearFault(void)
{
    g_ctrl.fault_latched = false;
    g_ctrl.integral = 0.0f;
    g_ctrl.prev_error = 0.0f;
    g_ctrl.last_angle_deg = read_encoder_deg();
    g_ctrl.last_tick_ms = HAL_GetTick();
    g_ctrl.last_motion_tick_ms = g_ctrl.last_tick_ms;
}

TurntableControlStatus_t MoveToAngle(float target_angle)
{
    float target_deg;
    uint32_t now_ms;
    float dt_sec;
    float current_deg;
    float error_deg;
    float derivative;
    float pid;
    float cmd_hz;
    float scale;
    TurntableDirection_t direction;
    TurntableControlStatus_t stall_status;

    if (!g_ctrl.initialized)
    {
        return TURNTABLE_STATUS_NOT_INITIALIZED;
    }

    if (g_ctrl.fault_latched)
    {
        return TURNTABLE_STATUS_STALL_FAULT;
    }

    target_deg = nearest_index_angle(target_angle);
    now_ms = HAL_GetTick();
    dt_sec = ((float)(now_ms - g_ctrl.last_tick_ms)) / 1000.0f;
    g_ctrl.last_tick_ms = now_ms;
    if (dt_sec <= 0.0f)
    {
        dt_sec = 0.001f;
    }

    current_deg = read_encoder_deg();
    error_deg = wrap_signed_deg(target_deg - current_deg);

    if (fabsf(error_deg) <= POSITION_TOLERANCE_DEG)
    {
        g_ctrl.integral = 0.0f;
        g_ctrl.prev_error = error_deg;
        g_ctrl.last_angle_deg = current_deg;
        g_ctrl.last_motion_tick_ms = now_ms;
        PowerSTEP01_SoftStop();
        return TURNTABLE_STATUS_REACHED;
    }

    g_ctrl.integral += error_deg * dt_sec;
    g_ctrl.integral = clampf(g_ctrl.integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);

    derivative = (error_deg - g_ctrl.prev_error) / dt_sec;
    g_ctrl.prev_error = error_deg;

    pid = (PID_KP * error_deg) + (PID_KI * g_ctrl.integral) + (PID_KD * derivative);
    scale = slowdown_scale(current_deg);
    cmd_hz = fabsf(pid) * scale;
    cmd_hz = clampf(cmd_hz, STEPCLK_MIN_HZ, STEPCLK_MAX_HZ);

    direction = (pid >= 0.0f) ? TURNTABLE_DIR_FORWARD : TURNTABLE_DIR_REVERSE;
    PowerSTEP01_ApplyStepClock(direction, (uint32_t)cmd_hz);

    stall_status = check_stall_and_guard((direction == TURNTABLE_DIR_FORWARD) ? cmd_hz : -cmd_hz,
                                         current_deg,
                                         now_ms);
    if (stall_status == TURNTABLE_STATUS_STALL_FAULT)
    {
        return stall_status;
    }

    return TURNTABLE_STATUS_IN_PROGRESS;
}
