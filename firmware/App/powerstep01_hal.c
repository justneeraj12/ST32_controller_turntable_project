#include "powerstep01_hal.h"

#include <stdbool.h>
#include <stddef.h>

extern SPI_HandleTypeDef hspi1;

#ifndef POWERSTEP_CS_GPIO_Port
#define POWERSTEP_CS_GPIO_Port GPIOB
#endif

#ifndef POWERSTEP_CS_Pin
#define POWERSTEP_CS_Pin GPIO_PIN_6
#endif

#define POWERSTEP_CMD_STEP_CLOCK      0x58U
#define POWERSTEP_CMD_SOFT_STOP       0xB0U
#define POWERSTEP_CMD_HARD_STOP       0xB8U
#define POWERSTEP_CMD_GET_STATUS      0xD0U

#define POWERSTEP_REG_ACC             0x05U
#define POWERSTEP_REG_DEC             0x06U
#define POWERSTEP_REG_KVAL_HOLD       0x09U
#define POWERSTEP_REG_KVAL_RUN        0x0AU
#define POWERSTEP_REG_KVAL_ACC        0x0BU
#define POWERSTEP_REG_KVAL_DEC        0x0CU
#define POWERSTEP_REG_OCD_TH          0x13U

#define POWERSTEP_DIR_REV             0x00U
#define POWERSTEP_DIR_FWD             0x01U

/* OCD threshold register uses 0.375A steps: I = 0.375 * (code + 1). */
#define POWERSTEP_OCD_TH_1P5A_CODE    0x03U

static bool g_stepclock_mode_active = false;
static TurntableDirection_t g_stepclock_direction = TURNTABLE_DIR_FORWARD;

static void powerstep_cs_low(void)
{
    HAL_GPIO_WritePin(POWERSTEP_CS_GPIO_Port, POWERSTEP_CS_Pin, GPIO_PIN_RESET);
}

static void powerstep_cs_high(void)
{
    HAL_GPIO_WritePin(POWERSTEP_CS_GPIO_Port, POWERSTEP_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef powerstep_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    HAL_StatusTypeDef status;

    if ((tx == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    powerstep_cs_low();
    status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, (uint16_t)len, HAL_MAX_DELAY);
    powerstep_cs_high();

    return status;
}

static HAL_StatusTypeDef powerstep_send_command(uint8_t cmd)
{
    uint8_t tx[1] = {cmd};
    uint8_t rx[1] = {0};
    return powerstep_transfer(tx, rx, sizeof(tx));
}

static HAL_StatusTypeDef powerstep_set_param_u8(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2] = {0};
    return powerstep_transfer(tx, rx, sizeof(tx));
}

static HAL_StatusTypeDef powerstep_set_param_u16(uint8_t reg, uint16_t value)
{
    uint8_t tx[3] = {reg, (uint8_t)((value >> 8) & 0xFFU), (uint8_t)(value & 0xFFU)};
    uint8_t rx[3] = {0};
    return powerstep_transfer(tx, rx, sizeof(tx));
}

static HAL_StatusTypeDef powerstep_enter_stepclock(TurntableDirection_t direction)
{
    uint8_t cmd = POWERSTEP_CMD_STEP_CLOCK |
                  ((direction == TURNTABLE_DIR_FORWARD) ? POWERSTEP_DIR_FWD : POWERSTEP_DIR_REV);

    HAL_StatusTypeDef status = powerstep_send_command(cmd);
    if (status == HAL_OK)
    {
        g_stepclock_mode_active = true;
        g_stepclock_direction = direction;
    }
    return status;
}

HAL_StatusTypeDef PowerSTEP01_Init_24V_Agilent(void)
{
    uint8_t tx_status[3] = {POWERSTEP_CMD_GET_STATUS, 0x00U, 0x00U};
    uint8_t rx_status[3] = {0};

    powerstep_cs_high();

    if (powerstep_set_param_u8(POWERSTEP_REG_KVAL_HOLD, 0x29U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (powerstep_set_param_u8(POWERSTEP_REG_KVAL_RUN, 0x80U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (powerstep_set_param_u8(POWERSTEP_REG_KVAL_ACC, 0x80U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (powerstep_set_param_u8(POWERSTEP_REG_KVAL_DEC, 0x80U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (powerstep_set_param_u16(POWERSTEP_REG_ACC, 0x0080U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (powerstep_set_param_u16(POWERSTEP_REG_DEC, 0x0080U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (powerstep_set_param_u8(POWERSTEP_REG_OCD_TH, POWERSTEP_OCD_TH_1P5A_CODE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Clear any stale alarms after register programming. */
    if (powerstep_transfer(tx_status, rx_status, sizeof(tx_status)) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

__weak void PowerSTEP01_SetStepClockFrequency(uint32_t step_clock_hz)
{
    (void)step_clock_hz;
}

void PowerSTEP01_ApplyStepClock(TurntableDirection_t direction, uint32_t step_clock_hz)
{
    if ((!g_stepclock_mode_active) || (direction != g_stepclock_direction))
    {
        if (powerstep_enter_stepclock(direction) != HAL_OK)
        {
            PowerSTEP01_HardStop();
            return;
        }
    }

    PowerSTEP01_SetStepClockFrequency(step_clock_hz);
}

void PowerSTEP01_SoftStop(void)
{
    (void)powerstep_send_command(POWERSTEP_CMD_SOFT_STOP);
    PowerSTEP01_SetStepClockFrequency(0U);
    g_stepclock_mode_active = false;
}

void PowerSTEP01_HardStop(void)
{
    (void)powerstep_send_command(POWERSTEP_CMD_HARD_STOP);
    PowerSTEP01_SetStepClockFrequency(0U);
    g_stepclock_mode_active = false;
}

HAL_StatusTypeDef PowerSTEP01_TestClosedLoop90deg(void)
{
    TurntableControlStatus_t status;
    uint32_t start_tick;

    if (Turntable_InitControl() != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (PowerSTEP01_Init_24V_Agilent() != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (powerstep_enter_stepclock(TURNTABLE_DIR_FORWARD) != HAL_OK)
    {
        return HAL_ERROR;
    }

    start_tick = HAL_GetTick();
    do
    {
        status = MoveToAngle(90.0f);
        if (status == TURNTABLE_STATUS_STALL_FAULT)
        {
            return HAL_ERROR;
        }
        if ((HAL_GetTick() - start_tick) > 10000U)
        {
            PowerSTEP01_HardStop();
            return HAL_TIMEOUT;
        }
        HAL_Delay(5U);
    } while (status != TURNTABLE_STATUS_REACHED);

    HAL_Delay(2000U);

    start_tick = HAL_GetTick();
    do
    {
        status = MoveToAngle(0.0f);
        if (status == TURNTABLE_STATUS_STALL_FAULT)
        {
            return HAL_ERROR;
        }
        if ((HAL_GetTick() - start_tick) > 10000U)
        {
            PowerSTEP01_HardStop();
            return HAL_TIMEOUT;
        }
        HAL_Delay(5U);
    } while (status != TURNTABLE_STATUS_REACHED);

    PowerSTEP01_SoftStop();
    return HAL_OK;
}
