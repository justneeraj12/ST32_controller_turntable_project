#include "turntable_test.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

/* Confirmed hardware map:
 * SPI1 on PA5/PA6/PA7 and manual NSS on PB6.
 */
#define POWERSTEP_CS_GPIO_Port            GPIOB
#define POWERSTEP_CS_Pin                  GPIO_PIN_6

#define POWERSTEP_CMD_MOVE                0x40U
#define POWERSTEP_CMD_GET_STATUS          0xD0U

#define POWERSTEP_DIR_CCW                 0x00U
#define POWERSTEP_DIR_CW                  0x01U

#define POWERSTEP_REG_ACC                 0x05U
#define POWERSTEP_REG_DEC                 0x06U
#define POWERSTEP_REG_KVAL_HOLD           0x09U
#define POWERSTEP_REG_KVAL_RUN            0x0AU
#define POWERSTEP_REG_KVAL_ACC            0x0BU
#define POWERSTEP_REG_KVAL_DEC            0x0CU

/* 10% and 50% of the 8-bit KVAL range (0..255). */
#define POWERSTEP_KVAL_HOLD_10PCT         0x1AU
#define POWERSTEP_KVAL_RUN_50PCT          0x80U
#define POWERSTEP_KVAL_ACC_50PCT          0x80U
#define POWERSTEP_KVAL_DEC_50PCT          0x80U

/* ACC/DEC parameter values requested for startup profile. */
#define POWERSTEP_ACC_PROFILE             0x0080U
#define POWERSTEP_DEC_PROFILE             0x0080U

/* STATUS register helpers. */
#define POWERSTEP_STATUS_HIZ_MASK         (1U << 0)
#define POWERSTEP_STATUS_MOT_MASK         (3U << 5)
#define POWERSTEP_STATUS_MOT_STOPPED      (0U << 5)

/* Move command uses 22-bit step count. Tune this constant to match mechanics. */
#define TURNTABLE_STEPS_PER_REV           4096U
#define TURNTABLE_90DEG_STEPS             (TURNTABLE_STEPS_PER_REV / 4U)
#define MOVE_TIMEOUT_MS                   12000U

static void powerstep_cs_low(void)
{
    HAL_GPIO_WritePin(POWERSTEP_CS_GPIO_Port, POWERSTEP_CS_Pin, GPIO_PIN_RESET);
}

static void powerstep_cs_high(void)
{
    HAL_GPIO_WritePin(POWERSTEP_CS_GPIO_Port, POWERSTEP_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef powerstep_spi_txrx(const uint8_t *tx, uint8_t *rx, size_t len)
{
    HAL_StatusTypeDef status;

    if ((tx == NULL) || (rx == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    powerstep_cs_low();
    status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, (uint16_t)len, HAL_MAX_DELAY);
    powerstep_cs_high();

    return status;
}

static HAL_StatusTypeDef powerstep_write_u8(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};
    uint8_t rx[2] = {0};
    return powerstep_spi_txrx(tx, rx, sizeof(tx));
}

static HAL_StatusTypeDef powerstep_write_u16(uint8_t reg, uint16_t value)
{
    uint8_t tx[3] = {reg, (uint8_t)((value >> 8) & 0xFFU), (uint8_t)(value & 0xFFU)};
    uint8_t rx[3] = {0};
    return powerstep_spi_txrx(tx, rx, sizeof(tx));
}

static HAL_StatusTypeDef powerstep_read_status(uint16_t *status_word)
{
    uint8_t tx[3] = {POWERSTEP_CMD_GET_STATUS, 0x00U, 0x00U};
    uint8_t rx[3] = {0};

    if (status_word == NULL)
    {
        return HAL_ERROR;
    }

    if (powerstep_spi_txrx(tx, rx, sizeof(tx)) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *status_word = (uint16_t)(((uint16_t)rx[1] << 8) | rx[2]);
    return HAL_OK;
}

static HAL_StatusTypeDef powerstep_move(uint8_t direction, uint32_t n_step)
{
    uint32_t step_22b = n_step & 0x003FFFFFU;
    uint8_t tx[4] = {
        (uint8_t)(POWERSTEP_CMD_MOVE | (direction & 0x01U)),
        (uint8_t)((step_22b >> 16) & 0x3FU),
        (uint8_t)((step_22b >> 8) & 0xFFU),
        (uint8_t)(step_22b & 0xFFU)};
    uint8_t rx[4] = {0};

    return powerstep_spi_txrx(tx, rx, sizeof(tx));
}

static HAL_StatusTypeDef powerstep_wait_motion_done(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        uint16_t status = 0;
        if (powerstep_read_status(&status) != HAL_OK)
        {
            return HAL_ERROR;
        }

        if ((status & POWERSTEP_STATUS_MOT_MASK) == POWERSTEP_STATUS_MOT_STOPPED)
        {
            return HAL_OK;
        }

        HAL_Delay(5U);
    }

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef TurntableTest_PowerSTEP01_Init24V(void)
{
    uint16_t status_word;

    if ((hspi1.Init.CLKPolarity != SPI_POLARITY_HIGH) ||
        (hspi1.Init.CLKPhase != SPI_PHASE_2EDGE))
    {
        printf("INIT_FAIL_SPI_MODE\r\n");
        return HAL_ERROR;
    }

    powerstep_cs_high();

    if (powerstep_write_u8(POWERSTEP_REG_KVAL_HOLD, POWERSTEP_KVAL_HOLD_10PCT) != HAL_OK)
    {
        printf("INIT_FAIL_KVAL_HOLD\r\n");
        return HAL_ERROR;
    }

    if (powerstep_write_u8(POWERSTEP_REG_KVAL_RUN, POWERSTEP_KVAL_RUN_50PCT) != HAL_OK)
    {
        printf("INIT_FAIL_KVAL_RUN\r\n");
        return HAL_ERROR;
    }

    if (powerstep_write_u8(POWERSTEP_REG_KVAL_ACC, POWERSTEP_KVAL_ACC_50PCT) != HAL_OK)
    {
        printf("INIT_FAIL_KVAL_ACC\r\n");
        return HAL_ERROR;
    }

    if (powerstep_write_u8(POWERSTEP_REG_KVAL_DEC, POWERSTEP_KVAL_DEC_50PCT) != HAL_OK)
    {
        printf("INIT_FAIL_KVAL_DEC\r\n");
        return HAL_ERROR;
    }

    if (powerstep_write_u16(POWERSTEP_REG_ACC, POWERSTEP_ACC_PROFILE) != HAL_OK)
    {
        printf("INIT_FAIL_ACC\r\n");
        return HAL_ERROR;
    }

    if (powerstep_write_u16(POWERSTEP_REG_DEC, POWERSTEP_DEC_PROFILE) != HAL_OK)
    {
        printf("INIT_FAIL_DEC\r\n");
        return HAL_ERROR;
    }

    /* Read status once to clear stale flags after boot/register writes. */
    if (powerstep_read_status(&status_word) != HAL_OK)
    {
        printf("INIT_FAIL_STATUS\r\n");
        return HAL_ERROR;
    }

    (void)status_word;
    return HAL_OK;
}

void Check_Status(void)
{
    uint16_t status = 0;

    if (powerstep_read_status(&status) != HAL_OK)
    {
        printf("SPI_ERROR\r\n");
        return;
    }

    if (((status & POWERSTEP_STATUS_HIZ_MASK) == 0U) &&
        ((status & POWERSTEP_STATUS_MOT_MASK) != POWERSTEP_STATUS_MOT_STOPPED))
    {
        printf("MOTOR_MOVING\r\n");
    }
    else
    {
        printf("MOTOR_HI_Z\r\n");
    }
}

HAL_StatusTypeDef Test_Turntable_90deg(void)
{
    printf("TEST_START\r\n");

    if (TurntableTest_PowerSTEP01_Init24V() != HAL_OK)
    {
        printf("TEST_ABORT_INIT\r\n");
        return HAL_ERROR;
    }

    /* MOVE (0x40 | DIR) command: rotate 90 degrees clockwise. */
    if (powerstep_move(POWERSTEP_DIR_CW, TURNTABLE_90DEG_STEPS) != HAL_OK)
    {
        printf("TEST_ABORT_MOVE_CW\r\n");
        return HAL_ERROR;
    }
    Check_Status();

    if (powerstep_wait_motion_done(MOVE_TIMEOUT_MS) != HAL_OK)
    {
        printf("TEST_ABORT_TIMEOUT_CW\r\n");
        return HAL_TIMEOUT;
    }

    HAL_Delay(2000U);

    /* Return back to 0 by moving the same amount counter-clockwise. */
    if (powerstep_move(POWERSTEP_DIR_CCW, TURNTABLE_90DEG_STEPS) != HAL_OK)
    {
        printf("TEST_ABORT_MOVE_CCW\r\n");
        return HAL_ERROR;
    }
    Check_Status();

    if (powerstep_wait_motion_done(MOVE_TIMEOUT_MS) != HAL_OK)
    {
        printf("TEST_ABORT_TIMEOUT_CCW\r\n");
        return HAL_TIMEOUT;
    }

    Check_Status();
    printf("TEST_DONE\r\n");
    return HAL_OK;
}
